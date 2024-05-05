// Simulation wiredly.v
// This module provide the definition of a zero ohm component (A, B).
//
// The applications of this component include:
//   . Normal operation of a jumper wire (data flowing in both directions)
//     This can corrupt data from DRAM to FPGA useful for verifying ECC function. 
//
// The component consists of 2 ports:
//    . Port A: One side of the pass-through switch
//    . Port B: The other side of the pass-through switch

// The model is sensitive to transactions on all ports.  Once a transaction
// is detected, all other transactions are ignored for that simulation time
// (i.e. further transactions in that delta time are ignored).
//
// Model Limitations and Restrictions:
//   Signals asserted on the ports of the error injector should not have
//   transactions occuring in multiple delta times because the model
//   is sensitive to transactions on port A, B ONLY ONCE during
//   a simulation time.  Thus, once fired, a process will
//   not refire if there are multiple transactions occuring in delta times.
//   This condition may occur in gate level simulations with
//   ZERO delays because transactions may occur in multiple delta times.

`timescale 1ps / 1ps

module wiredly # 
(
    parameter Delay_g = 0,
    parameter Delay_rd = 0,
    parameter ERR_INSERT = "OFF"
)(
    inout A,
    inout B,
    input reset,
    input phy_init_done
);

    reg A_r;
    reg B_r;
    reg B_inv ;
    reg line_en;

    reg B_nonX;

    assign A = A_r;
    assign B = B_r;

    always @ (*)
    begin
        if (B === 1'bx)
            B_nonX <= $random;
        else
            B_nonX <= B;
    end
   
    always@(*)
    begin
        if((B_nonX == 'b1) || (B_nonX == 'b0))
            B_inv <= #0 ~B_nonX ;
        else
            B_inv <= #0 'bz ;
    end
   
    always @(*) 
    begin
        if (!reset) 
        begin
            A_r <= 1'bz;
            B_r <= 1'bz;
            line_en <= 1'b0;
        end 
        else 
        begin
            if (line_en) 
            begin
                B_r <= 1'bz;
                if ((ERR_INSERT == "ON") & (phy_init_done))
                    A_r <= #Delay_rd B_inv;
                else
                    A_r <= #Delay_rd B_nonX;
            end 
            else 
            begin
                B_r <= #Delay_g A;
                A_r <= 1'bz;
            end
        end
    end

    always @(A or B) 
    begin
        if (!reset) 
        begin
            line_en <= 1'b0;
        end 
        else if (A !== A_r) 
        begin
            line_en <= 1'b0;
        end 
        else if (B_r !== B) 
        begin
            line_en <= 1'b1;
        end 
        else 
        begin
            line_en <= line_en;
        end
    end

endmodule

