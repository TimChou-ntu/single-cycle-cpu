// Your code
module CHIP(clk,
              rst_n,
              // For mem_D
              mem_wen_D,
              mem_addr_D,
              mem_wdata_D,
              mem_rdata_D,
              // For mem_I
              mem_addr_I,
              mem_rdata_I);

  input         clk, rst_n ;
  // For mem_D
  output        mem_wen_D  ;
  output [31:0] mem_addr_D ;
  output [31:0] mem_wdata_D;
  input  [31:0] mem_rdata_D;
  // For mem_I
  output [31:0] mem_addr_I ;
  input  [31:0] mem_rdata_I;

  //---------------------------------------//
  // Do not modify this part!!!            //
  // Exception: You may change wire to reg //
  reg    [31:0] PC          ;              //
  wire   [31:0] PC_nxt      ;              //
  wire          regWrite    ;              //
  wire   [ 4:0] rs1, rs2, rd;              //
  wire   [31:0] rs1_data    ;              //
  wire   [31:0] rs2_data    ;              //
  wire   [31:0] rd_data     ;              //
  //---------------------------------------//

  // Todo: other wire/reg
  // I
  wire [31:0] Idata;
  wire [31:0] I_addr_add4, I_addr_branch, I_addr_jalr;
  // ALU
  wire [3:0] alu_ctrl;
  wire [31:0] alu_src1, alu_src2;
  wire alu_zero;
  wire [63:0] alu_out;
  // control
  wire [31:0] imm;
  wire jal, jalr, beq, bneq, memW, mem2reg,regW,alusrc,aluctrl;
  //---------------------------------------//
  // Do not modify this part!!!            //
  reg_file reg0(                           //
             .clk(clk),                           //
             .rst_n(rst_n),                       //
             .wen(regWrite),                      //
             .a1(rs1),                            //
             .a2(rs2),                            //
             .aw(rd),                             //
             .d(rd_data),                         //
             .q1(rs1_data),                       //
             .q2(rs2_data));                      //
  //---------------------------------------//
  Control control(.instruction(I_data),
                  .jal(jal),
                  .jalr(jalr),
                  .beq(beq),
                  .bneq(bneq),
                  .memwrite(memW),
                  .mem2reg(mem2reg),
                  .regwrite(regW),
                  .alusrc(alusrc),
                  .ALUctrl(aluctrl),
                  imm(imm));
  ALU alu(.ctrl(alu_ctrl),
          .x(alu_src1),
          .y(alu_src2),
          .zero(alu_zero),
          .out(alu_out));
  // Todo: any combinational/sequential circuit
  // I
  assign I_data = mem_rdata_I[31:0];  // not sure
  assign mem_addr_I = PC;
  assign I_addr_add4 = PC + 4;
  assign I_addr_branch = PC + imm[31:0];
  assign I_addr_jalr = imm[31:0] + rs1_data[31:0]; // not sure
  // ALU
  assign alu_src1 = rs1_data;
  assign alu_src2 = (alusrc)? imm : rs2_data;
  // register file
  assign regWrite = regW;
  assign rs1 = I_data[19:15];
  assign rs2 = I_data[24:20];
  assign rd = I_data[11:7];
  assign rd_data = (jal|jalr)? I_addr_add4[31:0] : (mem2reg)? mem_rdata_D : alu_out;
  // D mem
  assign mem_wen_D = memW;
  assign mem_wdata_D = rs2_data;
  assign mem_addr_D = I_addr_jalr;

  always @(*)
  begin
    if(jalr)
      PC_nxt = I_addr_jalr;
    else if (jal | (beq&alu_zero) | (bneq&(~alu_zero)))
      PC_nxt = I_addr_branch;
    else
      PC_nxt = I_addr_add4;
  end


  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      PC <= 32'h00010000; // Do not modify this value!!!

    end
    else
    begin
      PC <= PC_nxt;

    end
  end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

  parameter BITS = 32;
  parameter word_depth = 32;
  parameter addr_width = 5;                   // 2^addr_width >= word_depth

  input clk, rst_n, wen;                      // wen: 0:read | 1:write
  input [BITS-1:0] d;                         // d = wdata
  input [addr_width-1:0] a1, a2, aw;          // a1 = raddr1, a2 = raddr2, aw = waddr

  output [BITS-1:0] q1, q2;                   // q1 = rdata_1, q2 = rdata_2

  reg [BITS-1:0] mem [0:word_depth-1];
  reg [BITS-1:0] mem_nxt [0:word_depth-1];

  integer i;

  assign q1 = mem[a1];
  assign q2 = mem[a2];

  always @(*)
  begin
    for (i=0; i<word_depth; i=i+1)
      mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
  end

  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      mem[0] <= 0;
      for (i=1; i<word_depth; i=i+1)
      begin
        case(i)
          32'd2:
            mem[i] <= 32'hbffffff0;
          32'd3:
            mem[i] <= 32'h10008000;
          default:
            mem[i] <= 32'h0;
        endcase
      end
    end
    else
    begin
      mem[0] <= 0;
      for (i=1; i<word_depth; i=i+1)
        mem[i] <= mem_nxt[i];
    end
  end
endmodule

module ALU(ctrl, x, y, zero, out);
  input [3:0] ctrl;
  input [31:0] x;
  input [31:0] y;
  output reg zero;
  output reg [63:0] out;
  always @(*)
  begin
    out = 63'b0;
    zero = (x == y);
    case(ctrl)
      0:
        out = (x + y); // not sure signed or unsigned
      1:
        out = (x - y);
      2:
        out = (x << y);
      3:
        out = (x < y)? 1:0;
      4:
        out = (x ^ y);
      5:
        out = (x >> y);
      6:
        out = (x >>> y);
      7:
        out = (x | y);
      8:
        out = (x & y);
    endcase
  end
endmodule

module Control(instruction, jal, jalr, beq, bneq, memwrite, mem2reg, regwrite, alusrc, ALUctrl, imm);
  input [31:0] instruction;
  output reg jal;
  output reg jalr;
  output reg beq;
  output reg bneq;
  output reg memwrite;
  output mem2reg;
  output reg regwrite;
  output reg alusrv;
  output reg [3:0] ALUctrl;
  output reg [31:0] imm;

  wire [6:0] opcode;
  wire bit_30;
  wire bit_25;
  wire [2:0] fun_3;

  parameter JAL = 7'b1101111;
  parameter JALR = 7'b1100111;
  parameter BRANCH = 7'b1100011;
  parameter I = 7'b0010011;
  parameter LD = 7'b0000011;
  parameter SD = 7'b0100011;
  parameter R = 7'b0110011;
  parameter AUIPC =7'b0010111;

  assign opcode = instruction[6:0];
  assign bit_30 = instruction[30];
  assign bit_25 = instruction[25];
  assign func_3 = instruction[14:12];
  assign mem2reg = (opcode==LD)? 1:0;

  always @(*)
  begin
    case(opcode)
      JAL:
      begin
        jal = 1'b1;
        jalr = 1'b0;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b0;
        regwrite = 1'b0;
        alusrc = 1'b0;
        ALUctrl = 4'd0;
        // not sure
        imm = {11'b0, instruction[31],instruction[19:12],instruction[20],instruction[30:21],1'b0};
      end
      JALR:
      begin
        jal = 1'b0;
        jalr = 1'b1;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b0;
        regwrite = 1'b0;
        alusrc = 1'b0;
        ALUctrl = 4'd0;
        // not sure
        imm = {20'b0, instruction[31:20]};
      end
      BRANCH:
      begin
        jal = 1'b0;
        jalr = 1'b0;
        if (func_3 == 3'b000)
        begin
          beq = 1'b1;
          bneq = 1'b0;
        end
        else
        begin          // func_3 == 001
          beq = 1'b0;
          bneq = 1'b1;
        end
        memwrite = 1'b0;
        regwrite = 1'b0;
        alusrc = 1'b0;
        ALUctrl = 4'd1;
        imm = {19'b0, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
      end
      I:
      begin
        jal = 1'b0;
        jalr = 1'b0;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b0;
        regwrite = 1'b1;
        alusrc = 1'b1;
        imm = {20'b0,imm[31:20]};
        if (func_ 3'b000)           // addi
          ALUctrl = 4'd0;
        else if (func_3 == 3'b010)  // slti
          ALUctrl = 4'd3;
        else
          ALUctrl = 4'd15;
      end
      LD:
      begin
        jal = 1'b0;
        jalr = 1'b0;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b0;
        regwrite = 1'b1;
        alusrc = 1'b1;
        imm = {20'b0, instruction[31:20]};
        ALUctrl = 4'd0;
      end
      SD:
      begin
        jal = 1'b0;
        jalr = 1'b0;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b1;
        regwrite = 1'b0;
        alusrc = 1'b1;
        imm = {20'b0, instruction[31:25], instruction[11:7]};
        ALUctrl = 4'd0;
      end
      R:
      begin
        jal = 1'b0;
        jalr = 1'b0;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b0;
        regwrite = 1'b1;
        alusrc = 1'b0;
        imm = {32'b0};
        if (func_3 == 3'b000)
        begin
          if (bit_30 == 1'b0 && bit_25 == 0)          // add
            ALUctrl = 4'd0;
          else if (bit_30 == 1'b1 && bit_25 == 0)     // sub
            ALUctrl = 4'd1;
          else                                        // mul
            ALUctrl = 4'd2;
        end
        else
          ALUctrl = 4'd15;
      end
      AUIPC:
      begin
        jal = 1'b0;
        jalr = 1'b0;
        beq = 1'b0;
        bneq = 1'b0;
        memwrite = 1'b0;
        regwrite = 1'b1;
        alusrc = 1'b0;
        imm = {instruction[31:12], 12'b0};
      end
    endcase
  end

endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
  // Todo: your HW2
  // Definition of ports

  input         clk, rst_n;
  input         valid;
  input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: and, 3: or
  output        ready;
  input  [31:0] in_A, in_B;
  output [63:0] out;

  // Definition of states
  parameter IDLE = 3'd0;
  parameter MUL  = 3'd1;
  parameter DIV  = 3'd2;
  parameter AND = 3'd3;
  parameter OR = 3'd4;
  parameter OUT  = 3'd5;

  // Todo: Wire and reg if needed
  reg  [ 2:0] state, state_nxt;
  reg  [ 4:0] counter, counter_nxt;
  reg  [63:0] shreg, shreg_nxt; //what is this for?
  reg  [31:0] alu_in, alu_in_nxt;//what is alu_in_nxt, counter_nxt for?
  reg  [32:0] alu_out;
  // Todo 5: Wire assignments
  assign ready= (state==OUT)?1'b1:1'b0;
  assign out = shreg;
  // Combinational always block
  // Todo 1: Next-state logic of state machine
  always @(*)
  begin
    state_nxt = state;
    case(state)
      IDLE:
      begin
        if(!valid)
        begin
          state_nxt=IDLE;
        end
        else
        begin
          if(mode==2'b00)
          begin
            state_nxt=MUL;
          end
          else if (mode==2'b01)
          begin
            state_nxt=DIV;
          end
          else if (mode==2'b10)
          begin
            state_nxt=AND;
          end
          else if (mode==2'b11)
          begin
            state_nxt=OR;
          end
        end
      end
      MUL :
      begin
        // in_A is multiplicand, in_B is multiplier
        //in_A is divisor, in_B is dividend
        if(counter==5'b11111)
          state_nxt=OUT;
        else
          state_nxt=MUL;
      end
      DIV :
      begin
        if(counter==5'b11111)
          state_nxt=OUT;
        else
          state_nxt=DIV;
      end
      AND :
        state_nxt = OUT;
      OR  :
        state_nxt = OUT;
      OUT :
        state_nxt = IDLE;
      default :
        state_nxt = IDLE;
    endcase
  end
  // Todo 2: Counter
  always @(*)
  begin
    counter_nxt=counter;
    case(state)
      MUL:
      begin
        if (counter<5'b11111)//do we need to write 32 in binary form?
        begin
          counter_nxt = counter + 6'b00001;

        end
      end
      DIV :
      begin
        if(counter<=6'b11111)
        begin
          counter_nxt=counter+6'b00001;

        end
      end
      default:
        counter_nxt = 0;
    endcase
  end
  // ALU input
  always @(*)
  begin
    case(state)
      IDLE:
      begin
        if (valid)
        begin
          alu_in_nxt = in_B;
        end
        else
        begin
          alu_in_nxt = 0;
        end
      end
      OUT :
        alu_in_nxt = 0;
      default:
        alu_in_nxt = alu_in;
    endcase
  end

  // Todo 3: ALU output
  always @(*)
  begin
    case(state)
      MUL:
      begin
        // if clock <32
        if(shreg[0]==1'b1)
        begin
          alu_out=alu_in+shreg[63:32]; //may use alu_in (?)
        end
        else
        begin
          alu_out=shreg[63:32];

        end
      end
      DIV :
      begin
        if(shreg[62:31]>alu_in)
        begin
          alu_out[32:1]=shreg[62:31]-alu_in;
          alu_out[0]=shreg[30];
        end
        else
        begin
          //shift-l
          alu_out=shreg[62:30];

        end
      end

      default:
        alu_out=0;
    endcase
  end
  //[31:0]sub = shreg_r[62:31] - alu_in_r ;
  // Todo 4: Shift register
  always @(*)
  begin
    case(state)
      IDLE:
      begin
        if (valid)
          //load data to the low 32 bits
          shreg_nxt[31:0]=in_A;
        else
          shreg_nxt[31:0]=0;
        shreg_nxt[63:32]=32'd0;
        //do we need to consider valid =0 ?

      end
      MUL :
      begin
        shreg_nxt[63:31]=alu_out;
        shreg_nxt[30:0]=shreg[31:1];
      end

      DIV:
      begin
        if(shreg[62:31]>=alu_in)
        begin
          //what if we shift here?
          shreg_nxt[63:31]=alu_out;
          //shreg_nxt=shreg<<1;
          shreg_nxt[30:1]=shreg[29:0];
          shreg_nxt[0]=1'b1;
          //if sub, add 1 after shift
        end
        else
        begin
          /*
          shreg[63:32]=alu_out[31:0];
          shreg_nxt=shreg<<1;
          */
          shreg_nxt[63:31]=alu_out;
          //shreg_nxt=shreg<<1;
          shreg_nxt[30:1]=shreg[29:0];
          shreg_nxt[0]=1'b0;
        end
      end
      AND:
      begin
        shreg_nxt[31:0]=shreg[31:0]& alu_in;
        shreg_nxt[63:32]=0;

      end
      OR:
      begin
        shreg_nxt[31:0]=shreg[31:0]| alu_in;
        shreg_nxt[63:32]=0;

      end
      OUT:
        shreg_nxt=0;
      default:
        shreg_nxt=shreg;
    endcase
  end

  // Todo: Sequential always block
  always @(posedge clk or negedge rst_n)
  begin
    if (!rst_n)
    begin
      state <= IDLE;
      alu_in<=0;
      counter<=0;

      shreg<=0;
    end
    else
    begin
      state <= state_nxt;
      counter<=counter_nxt;
      alu_in<=alu_in_nxt;
      shreg<=shreg_nxt;

    end
  end
endmodule
