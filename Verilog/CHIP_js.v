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
  input  [31:0] mem_rdata_I; // instr_code

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

  // Todo: any combinational/sequential circuit

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
  parameter addr_width = 5; // 2^addr_width >= word_depth

  input clk, rst_n, wen; // wen: 0:read | 1:write
  input [BITS-1:0] d;
  input [addr_width-1:0] a1, a2, aw;

  output [BITS-1:0] q1, q2;

  reg [BITS-1:0] mem [0:word_depth-1];
  reg [BITS-1:0] mem_nxt [0:word_depth-1];

  integer i;

  assign q1 = mem[a1];
  assign q2 = mem[a2];

  always @(*)
  begin
    for (i=0; i<word_depth; i=i+1)
      mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
  end-

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

module Controller( Opcode, Branch, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, ALUOp );


  input [6:0] Opcode;
  output Branch, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite;
  output [1:0] ALUOp;


  reg Branch, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite;
  reg [1:0] ALUOp;

  // Use Opcode to determine operation
  always@(*)
  case(Opcode)
    7'b0000011 :
    begin                  // lw
      MemtoReg = 1'b1;
      MemWrite = 1'b0;
      MemRead = 1'b1;
      ALUSrc = 1'b1;
      Branch = 1'b0;
      RegWrite = 1'b1;
      ALUOp = 2'b00;
    end
    7'b0100011 :
    begin                  // sw
      MemtoReg = 1'b0; //how to represent don't care ?
      MemWrite = 1'b1;
      MemRead = 1'b0;
      ALUSrc = 1'b1;
      Branch = 1'b0;
      RegWrite = 1'b0;
      ALUOp = 2'b00;
    end
    7'b0010011 :
    begin            // I-type Operation
      MemtoReg = 1'b0;
      MemWrite = 1'b0;
      MemRead = 1'b0;
      ALUSrc = 1'b1;
      RegWrite = 1'b1;
      ALUOp = 2'b00;
    end
    7'b0110011 :
    begin                   // R-type Operation
      MemtoReg = 1'b0;
      MemWrite = 1'b0;
      MemRead = 1'b0;
      ALUSrc = 1'b0;
      Branch = 1'b0;
      RegWrite = 1'b1;
      ALUOp = 2'b10;
    end
    7'b1100011:
    begin                   // branch
      MemtoReg = 1'b0; //don't care
      MemWrite = 1'b0;
      MemRead = 1'b0;
      ALUSrc = 1'b0;
      Branch = 1'b1;
      RegWrite = 1'b0;
      ALUOp = 2'b01;
    end
    7'b1101111:
    begin                   // jal

    end
    7'b1100111:
    begin                   //jalr

    end
    7'b0010111:
    begin                   // auipc

    end
    default    :
    begin                   // Set all signals to 0
      MemtoReg = 1'b0;
      MemWrite = 1'b0;
      MemRead = 1'b0;
      ALUSrc = 1'b0;
      Branch = 1'b0;
      RegWrite = 1'b0;
      ALUOp = 2'b00;
    end

  endcase

endmodule

module ALUControl( ALUOp, Funct7, Funct3, Operation );


  input  [1:0] ALUOp;
  input  [6:0] Funct7;
  input  [2:0] Funct3;
  output [3:0] Operation;

  reg [3:0] Operation;

  always@(*)
  begin
    case({Funct3,ALUOp})
      5'b11110 :
        Operation = 4'b0000;     // AND
      5'b11010 :
        Operation = 4'b0001;     // OR
      5'b10010 :
        Operation = 4'b1100;     // NOR
      5'b01010 :
        Operation = 4'b0111;     // slt
      5'b00010 :
        if(Funct7 == 7'h00)
          Operation = 4'b0010; // add
        else if (Funct7 == 7'h20)
          Operation = 4'b0110; // sub
      5'b01000 :
        Operation = 4'b0111;     // slti
      5'b00000 :
        Operation = 4'b0010;     // addi
      5'b01000 :
        Operation = 4'b0010;     // lw/sw
      default  :
        Operation = 4'b0000;     // default AND
    endcase
  end

endmodule

// alu
module alu( in_A, in_B, ALU_Operation, ALU_Out, Carry_Out, Zero, Overflow);
  input  [31:0] in_A;
  input  [31:0] in_B;
  input  [3:0] ALU_Operation;
  output [31:0] ALU_Out;
  output reg Carry_Out;
  output Zero;
  output reg Overflow = 1'b0;
  reg [31:0] ALU_Result;
  reg [32:0] temp;
  reg [32:0] twos_com;

  assign ALU_Out = ALU_Result;
  assign Zero    = ( ALU_Result == 0 );

  always @(*)
  begin
    Overflow  = 1'b0;
    Carry_Out = 1'b0;

    case(ALU_Operation)
      4'b0000: //and
        ALU_Result = in_A & in_B;

      4'b0001: //or
        ALU_Result = in_A | in_B;

      4'b0010: //addi
      begin
        ALU_Result = $signed(in_A) + $signed(in_B);
        temp = { 1'b0 , in_A } + { 1'b0 , in_B };
        Carry_Out = temp[32];
        if ( (in_A[31] & in_B[31] & ~ALU_Out[31]) |
             (~in_A[31] & ~in_B[31] & ALU_Out[31] ))
          Overflow = 1'b1;
        else
          Overflow = 1'b0;
      end

      4'b0110:  //sub
      begin
        ALU_Result = $signed(in_A) - $signed(in_B);
        twos_com   = ~(in_B) + 1'b1;

        if( (in_A[31] & twos_com[31] & ~ALU_Out[31]) |
            (~in_A[31] & ~twos_com[31] & ALU_Out[31]) )
          Overflow = 1'b1;
        else
          Overflow = 1'b0;

      end

      4'b0111: //slti
        ALU_Result = ($signed(in_A) < $signed(in_B))?32'd1:32'd0;

      4'b1100: //nor
        ALU_Result = ~(in_A | in_B);

      4'b1111: //Comparison
        ALU_Result = ( in_A == in_B )?32'd1:32'd0;

      default:
        ALU_Result = in_A + in_B;
    endcase
  end
endmodule

//immediate generator
module ImmGen( InstCode, ImmOut  );

  input [31:0] InstCode;
  output [31:0] ImmOut;
  reg [31:0] ImmOut;
  always@(InstCode)
  begin
    case(InstCode[6:0]) //check opcode
      7'b0000011 : //lw
        ImmOut = {InstCode[31] ? {20{1'b1}} : 20'b0, InstCode[31:20]};
      7'b0010011 : // I-type
        ImmOut = {InstCode[31] ? {20{1'b1}} : 20'b0, InstCode[31:20]};
      7'b0100011 : //sw
        ImmOut = {InstCode[31] ? {20{1'b1}} : 20'b0, InstCode[31:25], InstCode[11:7]};
      7'b0010111 : //auipc
        ImmOut = {InstCode[31:12], 12'b0};
      default :
        ImmOut = 32'b0;
    endcase
  end

endmodule

//mux
module MUX(D1, D2, S, Y );
  input         S;
  input  [31:0] D1, D2;
  output [31:0] Y;

  // If S is active then Y = D2
  assign Y = S ? D2:D1;

endmodule

//instruction memory
module InstructionMem(addr, instruction );

  input  [ 7:0] addr;
  output wire [31:0] instruction;
  reg [31:0] memory [63:0];

  initial
  begin
    memory[0] = 32'b ;
  end
  assign instruction = memory[addr[7:2]];

endmodule

//data memory
module DataMem( MemRead, MemWrite, addr, write_data, read_data );
  input         MemRead, MemWrite;
  input  [ 8:0] addr;
  input  [31:0] write_data;
  output [31:0] read_data;

  reg  [31:0] memory [127:0];
  wire [31:0] read_data;
  integer i;

  //initialize all 128 registers to zero
  initial
  begin
    for(i=0; i<128; i=i+1)
    begin
      memory[i] = 32'b0;
    end
  end

  always@(*)
  begin
    if(MemWrite)
      memory[addr] <= write_data;
  end

  assign read_data = MemRead ? memory[addr] : 32'b0;

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
