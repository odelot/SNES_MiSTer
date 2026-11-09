// Shared SDRAM SNI port: host PC (sni.sv, port A) and RA mirror (port B).
//
// sdram.sv was written for a single SNI client. It holds one pending request,
// and that request is cleared whenever an SNI access shows as started, so a
// request that arrives while another SNI access is in flight is dropped. Both
// clients also see the same sni_ready and sni_dout. Switching the port on
// ra_active left both failure modes open:
//   - a host read in flight when the RA scan began handed its ready and its
//     byte to the RA, so one address read a wrong value for that frame;
//   - during the RA window, the host took RA completions as its own: it got
//     RA bytes back, and its writes were silently lost.
//
// This arbiter issues one access at a time and holds the other client's
// request until the port is free. Each client gets its own ready and a copy of
// its own data. Both keep their existing handshake: ready is low from the cycle
// after a request until that client's own access completes.
module ra_sni_arb
(
	input             clk,      // clk_sys
	input             reset,

	// Port A: host PC (sni.sv), byte access
	input      [24:0] a_addr,
	input       [7:0] a_din,
	input             a_rd,
	input             a_wr,
	output reg  [7:0] a_q,
	output            a_ready,

	// Port B: RA mirror, reads only
	input      [24:0] b_addr,
	input             b_word,
	input             b_rd,
	output reg [15:0] b_dout,
	output            b_ready,

	// sdram.sv SNI port
	output     [24:0] s_addr,
	output     [15:0] s_din,
	output            s_word,
	output reg        s_rd,
	output reg        s_wr,
	input             s_ready,
	input      [15:0] s_dout
);

reg        a_pend = 0, a_pend_wr = 0, b_pend = 0;
reg        busy = 0, accepted = 0, owner_b = 0, last_b = 0;
reg [11:0] wdog = 0;

// The owner's address stays on the port until its access completes, so
// sdram.sv latches it at whatever cycle the access starts and selects the
// byte of sni_dout for it.
assign s_addr = owner_b ? b_addr : a_addr;
assign s_din  = owner_b ? 16'h0000 : {8'h00, a_din};
assign s_word = owner_b ? b_word : 1'b0;

assign a_ready = ~(a_rd | a_wr | a_pend | (busy & ~owner_b));
assign b_ready = ~(b_rd | b_pend | (busy &  owner_b));

// Round robin when both wait.
wire pick_a = a_pend & (~b_pend | last_b);
wire pick_b = b_pend & ~pick_a;

always @(posedge clk) begin
	s_rd <= 1'b0;
	s_wr <= 1'b0;

	if (reset) begin
		a_pend  <= 1'b0;
		b_pend  <= 1'b0;
		busy    <= 1'b0;
		owner_b <= 1'b0;
		last_b  <= 1'b0;
	end
	else begin
		if (a_rd | a_wr) begin
			a_pend    <= 1'b1;
			a_pend_wr <= a_wr;
		end
		if (b_rd) b_pend <= 1'b1;

		if (!busy) begin
			if (pick_a | pick_b) begin
				owner_b  <= pick_b;
				last_b   <= pick_b;
				busy     <= 1'b1;
				accepted <= 1'b0;
				wdog     <= 12'd0;
				s_rd     <= pick_b | ~a_pend_wr;
				s_wr     <= pick_a &  a_pend_wr;
			end
		end
		else begin
			wdog <= wdog + 12'd1;
			// sdram.sv drops sni_ready when it sees the request edge; only a
			// rise after that is this access completing.
			if (!s_ready) accepted <= 1'b1;
			// The watchdog only frees the port if a request edge is ever lost;
			// a real access completes in well under 100 cycles.
			if ((accepted && s_ready) || &wdog) begin
				busy <= 1'b0;
				if (owner_b) begin
					b_pend <= 1'b0;
					b_dout <= s_dout;
				end
				else begin
					a_pend <= 1'b0;
					a_q    <= s_dout[7:0];
				end
			end
		end
	end
end

endmodule
