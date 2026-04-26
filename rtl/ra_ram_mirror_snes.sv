// RetroAchievements RAM Mirror for SNES — Option C: Selective Address Reading
//
// Each VBlank, reads a list of specific addresses from DDRAM (written by ARM),
// fetches the byte values from SDRAM (WRAM) or BSRAM, and writes them back
// to DDRAM for the ARM to read via rcheevos.
//
// Typically ~185 addresses for a game like Super Mario World, taking ~30µs
// per frame. This is frame-accurate with zero visual impact.
//
// DDRAM Layout (at DDRAM_BASE, ARM phys 0x3D000000):
//   [0x00000] Header:   magic(32) + 0(8) + flags(8) + 0(16)
//   [0x00008] Frame:    frame_counter(32) + bsram_size(32)
//
//   [0x40000] AddrReq:  addr_count(32) + request_id(32)       (ARM → FPGA)
//   [0x40008] Addrs:    addr[0](32) + addr[1](32), ...        (2 per 64-bit word)
//
//   [0x48000] ValResp:  response_id(32) + response_frame(32)  (FPGA → ARM)
//   [0x48008] Values:   val[0..7](8b each), val[8..15], ...   (8 per 64-bit word)

module ra_ram_mirror_snes #(
	parameter [28:0] DDRAM_BASE = 29'h07A00000,  // ARM phys 0x3D000000 >> 3
	// Debug: bypass SNI/BSRAM reads, write addr-based pattern instead
	// Set to 1 to test DDRAM pipeline without relying on SDRAM reads
	// Expected value for each address = addr[7:0] ^ 8'hA5
	parameter BYPASS_SNI = 0
)(
	input             clk,           // clk_sys (~21.477 MHz)
	input             reset,
	input             vblank,

	// SDRAM SNI read interface (for WRAM)
	output reg [24:0] sni_addr,
	output reg        sni_rd_req,
	input      [15:0] sni_dout,
	input             sni_ready,
	output reg        sni_word,

	// BSRAM read interface
	output reg [17:0] bsram_addr,
	input       [7:0] bsram_dout,
	output reg        bsram_rd,
	input             bsram_ready,

	// BSRAM actual size (bytes, from cart header ram_mask+1)
	input      [17:0] bsram_size,

	// DDRAM write interface (toggle req/ack)
	output reg [28:0] ddram_wr_addr,
	output reg [63:0] ddram_wr_din,
	output reg  [7:0] ddram_wr_be,
	output reg        ddram_wr_req,
	input             ddram_wr_ack,

	// DDRAM read interface (toggle req/ack)
	output reg [28:0] ddram_rd_addr,
	output reg        ddram_rd_req,
	input             ddram_rd_ack,
	input      [63:0] ddram_rd_dout,

	// Status
	output reg        active,
	output reg [31:0] dbg_frame_counter
);

// ======================================================================
// Constants
// ======================================================================
localparam [28:0] ADDRLIST_BASE = DDRAM_BASE + 29'h8000;  // byte offset 0x40000 / 8
localparam [28:0] VALCACHE_BASE = DDRAM_BASE + 29'h9000;  // byte offset 0x48000 / 8
localparam [31:0] WRAM_LIMIT    = 32'h20000;              // 128KB boundary
localparam [12:0] MAX_ADDRS     = 13'd4096;

// ======================================================================
// Clock domain crossing synchronizers
// Note: sni_ready does NOT need a synchronizer because clk_sys and
// clk_mem come from the same PLL with a 4:1 ratio (synchronous clocks).
// The original sni.sv module also reads sni_ready directly.
// ======================================================================
reg dwr_ack_s1, dwr_ack_s2;
reg drd_ack_s1, drd_ack_s2;
always @(posedge clk) begin
	dwr_ack_s1 <= ddram_wr_ack; dwr_ack_s2 <= dwr_ack_s1;
	drd_ack_s1 <= ddram_rd_ack; drd_ack_s2 <= drd_ack_s1;
end

// ======================================================================
// VBlank edge detection
// ======================================================================
reg vblank_prev;
wire vblank_rising = vblank & ~vblank_prev;
always @(posedge clk) vblank_prev <= vblank;

// ======================================================================
// State machine
// ======================================================================
localparam S_IDLE        = 5'd0;
localparam S_DD_WR_WAIT  = 5'd1;   // Generic: wait DDRAM write ack
localparam S_DD_RD_WAIT  = 5'd2;   // Generic: wait DDRAM read ack
localparam S_READ_HDR    = 5'd3;   // Issue DDRAM read: addr list header
localparam S_PARSE_HDR   = 5'd4;   // Parse addr_count + request_id
localparam S_READ_PAIR   = 5'd5;   // Issue DDRAM read: address pair
localparam S_PARSE_ADDR  = 5'd6;   // Extract address from cached word
localparam S_DISPATCH    = 5'd7;   // Route to WRAM or BSRAM fetch
localparam S_FETCH_WRAM  = 5'd8;   // Issue SDRAM SNI read
localparam S_WRAM_WAIT   = 5'd9;   // Wait for SDRAM ready
localparam S_FETCH_BSRAM = 5'd10;  // Issue BSRAM read
localparam S_BSRAM_WAIT  = 5'd11;  // 1-cycle BSRAM latency
localparam S_STORE_VAL   = 5'd12;  // Store byte in collect buffer
localparam S_FLUSH_BUF   = 5'd13;  // Write collect buffer to DDRAM
localparam S_WRITE_RESP  = 5'd14;  // Write response header
localparam S_WR_HDR0     = 5'd15;  // Write main header word 0 (busy=0)
localparam S_WR_HDR1     = 5'd16;  // Write main header word 1 (frame+bsram)
localparam S_WR_DBG      = 5'd17;  // Write debug counters word 1
localparam S_WR_DBG2     = 5'd18;  // Write debug counters word 2

reg [4:0]  state;
reg [4:0]  return_state;

reg [31:0] frame_counter;
always @(posedge clk) dbg_frame_counter <= frame_counter;

reg [63:0] rd_data;         // Captured DDRAM read data
reg [31:0] req_count;       // Number of addresses in request
reg [31:0] req_id;          // Request ID from ARM
reg [12:0] addr_idx;        // Current address index (0..req_count-1)
reg [63:0] addr_word;       // Cached DDRAM word containing 2 addresses
reg [31:0] cur_addr;        // Current address being fetched
reg        cur_addr_lsb;    // LSB of WRAM address (byte within word)
reg [63:0] collect_buf;
reg  [3:0] collect_cnt;     // 0-8 bytes in buffer
reg [12:0] val_word_idx;    // DDRAM word index for value writes

reg        sni_phase;       // 0=pulse sent (skip first cycle), 1=polling sni_ready
reg [15:0] timeout;
reg  [7:0] fetch_byte;

// Debug counters (reset each frame)
reg [15:0] dbg_ok_cnt;       // SNI reads completed normally
reg [15:0] dbg_timeout_cnt;  // SNI reads that timed out
reg [15:0] dbg_first_dout;   // raw sni_dout from first good read
reg        dbg_first_cap;    // first good read captured flag
reg [15:0] dbg_max_timeout;  // max timeout value seen in a good read
reg  [7:0] dbg_dispatch_cnt; // entries to S_DISPATCH
reg [15:0] dbg_wram_cnt;     // entries to S_FETCH_WRAM
reg [15:0] dbg_bsram_cnt;    // entries to S_FETCH_BSRAM
reg [15:0] dbg_first_addr;   // first cur_addr from S_DISPATCH

// ======================================================================
// Main state machine
// ======================================================================
always @(posedge clk) begin
	bsram_rd <= 1'b0;

	if (reset) begin
		state        <= S_IDLE;
		active       <= 1'b0;
		frame_counter <= 32'd0;
		sni_rd_req   <= 1'b0;
		sni_word     <= 1'b0;
		ddram_wr_req <= dwr_ack_s2;
		ddram_rd_req <= drd_ack_s2;
	end
	else begin
		case (state)

		// =============================================================
		// IDLE: Wait for VBlank rising edge
		// =============================================================
		S_IDLE: begin
			active   <= 1'b0;
			sni_word <= 1'b0;
			if (vblank_rising) begin
				active <= 1'b1;
				// Reset debug counters for this frame
				dbg_ok_cnt      <= 16'd0;
				dbg_timeout_cnt <= 16'd0;
				dbg_first_cap   <= 1'b0;
				dbg_first_dout  <= 16'd0;
				dbg_max_timeout <= 16'd0;
				dbg_dispatch_cnt <= 8'd0;
				dbg_wram_cnt    <= 16'd0;
				dbg_bsram_cnt   <= 16'd0;
				dbg_first_addr  <= 16'd0;
				// Write header with busy=1
				ddram_wr_addr <= DDRAM_BASE;
				ddram_wr_din  <= {16'h0100, 8'h01, 8'd0, 32'h52414348};
				ddram_wr_be   <= 8'hFF;
				ddram_wr_req  <= ~ddram_wr_req;
				return_state  <= S_READ_HDR;
				state         <= S_DD_WR_WAIT;
			end
		end

		// =============================================================
		// Generic DDRAM write wait
		// =============================================================
		S_DD_WR_WAIT: begin
			if (ddram_wr_req == dwr_ack_s2)
				state <= return_state;
		end

		// =============================================================
		// Generic DDRAM read wait — capture data
		// =============================================================
		S_DD_RD_WAIT: begin
			if (ddram_rd_req == drd_ack_s2) begin
				rd_data <= ddram_rd_dout;
				state   <= return_state;
			end
		end

		// =============================================================
		// Read address list header from DDRAM
		// =============================================================
		S_READ_HDR: begin
			ddram_rd_addr <= ADDRLIST_BASE;
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_HDR;
			state         <= S_DD_RD_WAIT;
		end

		// =============================================================
		// Parse header: extract addr_count and request_id
		// =============================================================
		S_PARSE_HDR: begin
			req_id <= rd_data[63:32];
			if (rd_data[31:0] == 32'd0) begin
				// No addresses — skip to response
				req_count <= 32'd0;
				state     <= S_WRITE_RESP;
			end else begin
				req_count    <= (rd_data[31:0] > {19'd0, MAX_ADDRS}) ?
				                {19'd0, MAX_ADDRS} : rd_data[31:0];
				addr_idx     <= 13'd0;
				collect_cnt  <= 4'd0;
				collect_buf  <= 64'd0;
				val_word_idx <= 13'd0;
				state        <= S_READ_PAIR;
			end
		end

		// =============================================================
		// Read address pair from DDRAM (2 addrs per 64-bit word)
		// =============================================================
		S_READ_PAIR: begin
			ddram_rd_addr <= ADDRLIST_BASE + 29'd1 + {16'd0, addr_idx[12:1]};
			ddram_rd_req  <= ~ddram_rd_req;
			return_state  <= S_PARSE_ADDR;
			state         <= S_DD_RD_WAIT;
		end

		// =============================================================
		// Extract current address from cached word
		// =============================================================
		S_PARSE_ADDR: begin
			if (!addr_idx[0]) begin
				// Even index: save word, use low 32 bits
				addr_word <= rd_data;
				cur_addr  <= rd_data[31:0];
			end else begin
				// Odd index: use high 32 bits from saved word
				cur_addr <= addr_word[63:32];
			end
			state <= S_DISPATCH;
		end

		// =============================================================
		// Route to WRAM (SDRAM) or BSRAM based on address
		// =============================================================
		S_DISPATCH: begin
			dbg_dispatch_cnt <= dbg_dispatch_cnt + 8'd1;
			if (!dbg_dispatch_cnt)
				dbg_first_addr <= cur_addr[15:0];
			if (BYPASS_SNI) begin
				// Debug bypass: write addr-based pattern, skip SDRAM/BSRAM
				fetch_byte <= cur_addr[7:0] ^ 8'hA5;
				state      <= S_STORE_VAL;
			end
			else if (cur_addr < WRAM_LIMIT) begin
				dbg_wram_cnt <= dbg_wram_cnt + 16'd1;
				state <= S_FETCH_WRAM;
			end
			else begin
				dbg_bsram_cnt <= dbg_bsram_cnt + 16'd1;
				state <= S_FETCH_BSRAM;
			end
		end

		// =============================================================
		// WRAM: byte-mode SDRAM SNI read (matches sni.sv protocol exactly)
		// Uses sni_word=0 (byte read) so sdram.sv handles byte selection.
		// VBlank gate: only start new reads while vblank is HIGH to avoid
		// SDRAM contention with PPU/CPU during active scanlines.
		// =============================================================
		S_FETCH_WRAM: begin
			if (vblank) begin
				// Byte address directly (sdram.sv uses addr[0] for byte select)
				sni_addr     <= {1'b1, 7'd0, cur_addr[16:0]};
				sni_word     <= 1'b0;     // byte mode (matches sni.sv)
				sni_rd_req   <= 1'b1;     // ONE cycle pulse
				sni_phase    <= 1'b0;
				timeout      <= 16'd0;
				state        <= S_WRAM_WAIT;
			end
			// else: stay in S_FETCH_WRAM until vblank goes HIGH again
		end

		S_WRAM_WAIT: begin
			sni_rd_req <= 1'b0;      // deassert immediately → one-cycle pulse
			timeout    <= timeout + 16'd1;
			if (!sni_phase) begin
				// First cycle: pulse just deasserted, sdram is clearing
				// sni_ready. Skip this cycle to avoid seeing stale HIGH.
				sni_phase <= 1'b1;
			end else begin
				// Poll sni_ready directly (synchronous clocks, no CDC needed)
				if (sni_ready) begin
					// Byte mode: sdram.sv returns selected byte in sni_dout[7:0]
					fetch_byte <= sni_dout[7:0];
					dbg_ok_cnt <= dbg_ok_cnt + 16'd1;
					if (!dbg_first_cap) begin
						dbg_first_dout <= sni_dout;
						dbg_first_cap  <= 1'b1;
					end
					if (timeout > dbg_max_timeout)
						dbg_max_timeout <= timeout;
					state <= S_STORE_VAL;
				end
			end
			// Timeout safety (~3ms)
			if (timeout >= 16'hFFFF) begin
				fetch_byte <= 8'd0;
				dbg_timeout_cnt <= dbg_timeout_cnt + 16'd1;
				state      <= S_STORE_VAL;
			end
		end

		// =============================================================
		// BSRAM: 1-cycle block RAM read
		// =============================================================
		S_FETCH_BSRAM: begin
			if (bsram_ready) begin
				bsram_addr <= cur_addr - 32'h20000;
				bsram_rd   <= 1'b1;
				state      <= S_BSRAM_WAIT;
			end
		end

		S_BSRAM_WAIT: begin
			fetch_byte <= bsram_dout;
			state      <= S_STORE_VAL;
		end

		// =============================================================
		// Store fetched byte in collect buffer, advance index
		// =============================================================
		S_STORE_VAL: begin
			case (collect_cnt[2:0])
				3'd0: collect_buf[ 7: 0] <= fetch_byte;
				3'd1: collect_buf[15: 8] <= fetch_byte;
				3'd2: collect_buf[23:16] <= fetch_byte;
				3'd3: collect_buf[31:24] <= fetch_byte;
				3'd4: collect_buf[39:32] <= fetch_byte;
				3'd5: collect_buf[47:40] <= fetch_byte;
				3'd6: collect_buf[55:48] <= fetch_byte;
				3'd7: collect_buf[63:56] <= fetch_byte;
			endcase
			collect_cnt <= collect_cnt + 4'd1;
			addr_idx    <= addr_idx + 13'd1;

			// Flush when buffer full (8 bytes) or last address
			if (collect_cnt == 4'd7 || (addr_idx + 13'd1 >= req_count[12:0])) begin
				state <= S_FLUSH_BUF;
			end
			// Next address routing
			else if (addr_idx[0]) begin
				// Was odd index → next is even → need new pair
				state <= S_READ_PAIR;
			end else begin
				// Was even index → next is odd → use cached high half
				state <= S_PARSE_ADDR;
			end
		end

		// =============================================================
		// Flush collect buffer to DDRAM value cache
		// =============================================================
		S_FLUSH_BUF: begin
			ddram_wr_addr <= VALCACHE_BASE + 29'd1 + {16'd0, val_word_idx};
			ddram_wr_din  <= collect_buf;
			ddram_wr_be   <= (collect_cnt == 4'd8) ? 8'hFF
			                 : ((8'd1 << collect_cnt[2:0]) - 8'd1);
			ddram_wr_req  <= ~ddram_wr_req;
			val_word_idx  <= val_word_idx + 13'd1;
			collect_cnt   <= 4'd0;
			collect_buf   <= 64'd0;

			if (addr_idx >= req_count[12:0]) begin
				return_state <= S_WRITE_RESP;
			end else if (!addr_idx[0]) begin
				// Even → need new DDRAM pair read
				return_state <= S_READ_PAIR;
			end else begin
				// Odd → use cached high half
				return_state <= S_PARSE_ADDR;
			end
			state <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Write response header: {response_frame, response_id}
		// =============================================================
		S_WRITE_RESP: begin
			ddram_wr_addr <= VALCACHE_BASE;
			ddram_wr_din  <= {frame_counter + 32'd1, req_id};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR0;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Write main header word 0: magic + flags=0 (not busy)
		// =============================================================
		S_WR_HDR0: begin
			ddram_wr_addr <= DDRAM_BASE;
			ddram_wr_din  <= {16'h0100, 8'h00, 8'd0, 32'h52414348};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_HDR1;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Write main header word 1: frame_counter + bsram_size
		// =============================================================
		S_WR_HDR1: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd1;
			ddram_wr_din  <= {{14'd0, bsram_size}, frame_counter + 32'd1};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			frame_counter <= frame_counter + 32'd1;
			return_state  <= S_WR_DBG;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Write debug word 1: {ver(8), dispatch(8), first_dout(16), timeout(16), ok(16)}
		// =============================================================
		S_WR_DBG: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd2;
			ddram_wr_din  <= {8'h0A, dbg_dispatch_cnt, dbg_first_dout, dbg_timeout_cnt, dbg_ok_cnt};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_WR_DBG2;
			state         <= S_DD_WR_WAIT;
		end

		// =============================================================
		// Write debug word 2: {first_addr(16), wram_cnt(16), bsram_cnt(16), max_timeout(16)}
		// =============================================================
		S_WR_DBG2: begin
			ddram_wr_addr <= DDRAM_BASE + 29'd3;
			ddram_wr_din  <= {dbg_first_addr, dbg_wram_cnt, dbg_bsram_cnt, dbg_max_timeout};
			ddram_wr_be   <= 8'hFF;
			ddram_wr_req  <= ~ddram_wr_req;
			return_state  <= S_IDLE;
			state         <= S_DD_WR_WAIT;
		end

		default: state <= S_IDLE;
		endcase
	end
end

endmodule
