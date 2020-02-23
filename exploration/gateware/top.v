/* Small test design actuating all IO on the iCEBreaker dev board. */

module top (
    input CLK,

	output LED1,
	output LED2,
);
    reg [26:0] counter = 0;

	always @(posedge CLK) begin
		counter <= counter + 1;
	end

	assign LED1 = ~counter[24];
    assign LED2 = ~counter[25];
    

endmodule