// Part 2 skeleton

module singleBoxDisplay
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
        KEY,
        SW,
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		PS2_CLK,
		PS2_DAT,
		LEDR
	);

	input			CLOCK_50;				//	50 MHz
	input   [9:0]   SW;
	input   [3:0]   KEY;

	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	wire resetn;
	assign resetn = KEY[0];
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "black.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn/plot
	// for the VGA controller, in addition to any other functionality your design may require.
    
    // Instansiate datapath
	// datapath d0(...);

    // Instansiate FSM control
    // control c0(...);


	 inout PS2_CLK;
	 inout PS2_DAT;
	 
	 output [9:0] LEDR;
	 
	 wire w1, a1, s1, d1, w2, a2, s2, d2, space, enter, n, m;
	 
	 keyboard_tracker #(.PULSE_OR_HOLD(0)) tester(
	     .clock(CLOCK_50),
		  .reset(KEY[0]),
		  .PS2_CLK(PS2_CLK),
		  .PS2_DAT(PS2_DAT),
		  .w(w1),
		  .a(a1),
		  .s(s1),
		  .d(d1),
		  .left(a2),
		  .right(d2),
		  .up(w2),
		  .down(s2),
		  .space(space),
		  .enter(enter),
		  .n(n),
		  .m(m)
		  );
	gameState game(CLOCK_50, resetn, n, m, enter, space, a1, d1, w1, s1, a2, d2, w2, s2, x, y, colour, writeEn);
	
	//gameState game(CLOCK_50, resetn, ~KEY[3], ~KEY[0], ~KEY[1], ~KEY[2], SW[3], SW[0], SW[2], SW[1], SW[9], SW[6], SW[8], SW[7], x, y, colour, writeEn);
	
	
endmodule

module gameState(clock, reset_n, isSingle, isMulti, gameStartButton, restart, left, right, up, down, p2left, p2right, p2up, p2down, outputX, outputY, outputC, plot);
	input clock, reset_n, isSingle, isMulti, gameStartButton, restart, left, right, up, down, p2left, p2right, p2up, p2down;
	
	reg [2:0] outputSelect;
	reg [3:0] currentstate, nextstate, wholePageCin;
	reg wholePageEnable, singleModeReset, multiModeReset, wholePageReset, borderEnable, delayReset, borderReset, snakeGoPrintScreen, snackReset, cleanScoreReset;
	
	wire isDead, isDead1, isDead2, goPrintSnack;
	wire [7:0] border_x, border_y, singleX, singleY, wholePageX, wholePageY, snackXOut, snackYOut, snackXCheck, snackYCheck, multiX, multiY, cleanX, cleanY;
	wire [2:0] borderColor, singleC, multiC, wholePageColorOut, snackCOut, cleanC, wholePageMultiC;
	wire singlePlot, multiP, changeLocation;
	
	wire [19:0] vgaDelayOut;
	
	output [7:0] outputX, outputY;
	output [2:0] outputC;
	output reg plot;

	localparam init           = 4'd0,
				  drawBorder     = 4'd1,
				  cleanScore     = 4'd2,
				  singlePlayer   = 4'd3,
				  printSnack     = 4'd4,
				  printSnackWait = 4'd5,
				  multiPlayer    = 4'd6,
				  gameOver       = 4'd7,
				  singleGameOver = 4'd8;
				  
	always @(*)
	begin
		case (currentstate)
		init: nextstate = (vgaDelayOut == 20'b00000000000000000000) ? drawBorder : init;
		drawBorder:
			begin
			if (isSingle) 
				nextstate = cleanScore;
			else if (isMulti)
				nextstate = multiPlayer;
			else
				nextstate = drawBorder;
			end
		cleanScore: nextstate = (vgaDelayOut == 20'b00000000000000000000) ? singlePlayer : cleanScore;
		singlePlayer:
			begin
			if (isDead)
				nextstate = singleGameOver;
			else if (goPrintSnack)
				nextstate = printSnack;
			else
				nextstate = singlePlayer;
			end
		printSnack: nextstate = (vgaDelayOut == 20'b00000000000000000000) ? printSnackWait : printSnack;
		printSnackWait: nextstate = singlePlayer;
		multiPlayer: nextstate = (| {isDead1, isDead2}) ? gameOver : multiPlayer;
		gameOver: nextstate = restart ? init : gameOver;
		singleGameOver: nextstate = restart ? init : singleGameOver;
		endcase
	end
	
	always @(posedge clock) 
	begin
		if(!reset_n)
			currentstate <= init;
		else
			currentstate <= nextstate;
	end
	
	always @(*)
	begin
		snackReset = 1'b1;
		outputSelect = 3'b000;
		plot = 1'b0;
		singleModeReset = 1'b0;
		multiModeReset = 1'b0;
		wholePageReset = 1'b0;
		wholePageEnable = 1'b0;
		wholePageCin = 3'b000;
		borderEnable = 1'b0;
		delayReset = 1'b0;
		borderReset = 1'b0;
		snakeGoPrintScreen = 1'b0;
		cleanScoreReset = 1'b0;
		case (currentstate)
			init:
				begin
				snackReset = 1'b0;
				delayReset = 1'b1;
				wholePageReset = 1'b1;
				wholePageEnable = 1'b1;
				wholePageCin = 3'b000;
				plot = 1'b1;
				outputSelect = 3'b000;
				end
			drawBorder:
				begin
				borderReset = 1'b1;
				borderEnable = 1'b1;
				plot = 1'b1;
				outputSelect = 3'b001;
				end
			cleanScore:
				begin
				delayReset = 1'b1;
				cleanScoreReset = 1'b1;
				plot = 1'b1;
				outputSelect = 3'b101;
				end
			singlePlayer:
				begin
				snakeGoPrintScreen = 1'b0;
				singleModeReset = 1'b1;
				plot = singlePlot;
				outputSelect = 3'b010;
				end
			printSnack:
				begin
				delayReset = 1'b1;
				plot = 1'b1;
				outputSelect = 3'b011;
				singleModeReset = 1'b1;
				end
			printSnackWait:
				begin
				snakeGoPrintScreen = 1'b1;
				singleModeReset = 1'b1;
				end
			multiPlayer:
				begin
				multiModeReset = 1'b1;
				outputSelect = 3'b100;
				plot = multiP;
				end
			gameOver:
				begin
				multiModeReset = 1'b0;
				wholePageReset = 1'b1;
				wholePageEnable = 1'b1;
				wholePageCin = wholePageMultiC;
				plot = 1'b1;
				outputSelect = 3'b000;
				end
			singleGameOver:
				begin
				wholePageReset = 1'b1;
				wholePageEnable = 1'b1;
				wholePageCin = 3'b100;
				plot = 1'b1;
				outputSelect = 3'b000;
				end
		endcase
	end
		
	vgaDelayCounter Delay(
		.clock(clock), 
		.reset_n(& {reset_n, delayReset}),
		.vgaDelayOut(vgaDelayOut)
		);
	
	border b0(clock, & {reset_n, borderReset}, 3'b111, border_x, border_y, borderColor, borderEnable);

	singlePlayerMode sp(clock, reset_n, singleModeReset, gameStartButton, left, right, up, down, singleX, singleY, singleC, singlePlot, isDead, goPrintSnack, snakeGoPrintScreen, snackXCheck, snackYCheck, changeLocation);

	
	multiPlayer mp(clock, reset_n, multiModeReset, gameStartButton, left, right, up, down, p2left, p2right, p2up, p2down, multiX, multiY, multiC, multiP, isDead1, isDead2, wholePageMultiC);
	
	snack s0(clock, snackReset, changeLocation, snackXOut, snackYOut, snackCOut, snackXCheck, snackYCheck);
	
	wholePagePrint wholePage(clock, & {reset_n, wholePageReset}, wholePageEnable, wholePageCin, wholePageX, wholePageY, wholePageColorOut);
	
	cleanP2Score clean(clock, cleanScoreReset, cleanX, cleanY, cleanC);
	
	myMux6to1 mX(wholePageX, border_x, singleX, snackXOut, multiX, cleanX, outputSelect, outputX);
	myMux6to1 mY(wholePageY, border_y, singleY, snackYOut, multiY, cleanY, outputSelect, outputY);
	myMux6to1 mC(wholePageColorOut, borderColor, singleC, snackCOut, multiC, cleanC, outputSelect, outputC);

endmodule


module singlePlayerMode(clock, reset_n, singleModeReset, gameStart, left, right, up, down, p1X, p1Y, p1C, p1Plot, isDead, goPrintSnack, snakeGoPrintScreen, snackX, snackY, changeLocation);
	input clock, reset_n, singleModeReset, gameStart, left, right, up, down, snakeGoPrintScreen;
	input [7:0] snackX, snackY;
	
	wire [7:0] p1HeadX, p1HeadY, p1BodyX, p1BodyY;
	
	output [7:0] p1X, p1Y;
	output [2:0] p1C;
	output p1Plot, isDead, goPrintSnack, changeLocation;

	snake p1(
		.clock(clock),
		.reset_n(& {reset_n, singleModeReset, ~isDead}),
		.initX(7'b0010011),
		.initY(7'b0010011),
		.selfColor(3'b011),
		.gameStartButton(gameStart),
		.left(left),
		.right(right),
		.up(up),
		.down(down),
		.xout(p1X),
		.yout(p1Y),
		.cout(p1C), 
		.plot(p1Plot),
		.headX(p1HeadX), 
		.headY(p1HeadY),
		.bodyX(p1BodyX),
		.bodyY(p1BodyY),
		.goPrintSnack(goPrintSnack),
		.snakeGoPrintScreen(snakeGoPrintScreen),
		.snackX(snackX), 
		.snackY(snackY),
		.changeLocation(changeLocation),
		.scoreyloc(7'b0001111)
		);
		
	collisionDetect singleP1(p1HeadX, p1HeadY, p1BodyX, p1BodyY, isDead);

endmodule

module multiPlayer(clock, reset_n, multiModeReset, gameStart, left, right, up, down, p2left, p2right, p2up, p2down, multiOutX, multiOutY, multiOutC, multiOutPlot, isDead1, isDead2, wholePageMultiC);
	input clock, reset_n, multiModeReset, gameStart, left, right, up, down, p2left, p2right, p2up, p2down;
	
	wire [7:0] HeadXp1, HeadYp1, BodyXp1, BodyYp1, HeadXp2, HeadYp2, BodyXp2, BodyYp2, p1X, p1Y, p1C, p2X, p2Y, p2C;
	wire [19:0] vgaDelayOut2, vgaDelayOut3;
	wire p1plot, p2Plot,goPrintSnake2, goPrintSnake1;
	
	reg currentstate, nextstate, select, p2Start, vgaDelayReset2, vgaDelayReset3;
	
	output [7:0] multiOutX, multiOutY;
	output [2:0] multiOutC;
	output multiOutPlot, isDead1, isDead2;
	output reg [2:0] wholePageMultiC;

	
	localparam p1Move = 1'b0,
				  p2Move = 1'b1;
				  
				  
	always @(*)
	begin
		case (currentstate)
		p1Move: nextstate = goPrintSnake2 ? p2Move : p1Move;
		p2Move: nextstate = goPrintSnake1 ? p1Move : p2Move;
		default: nextstate = p1Move;
		endcase
	end
	
	always @(posedge clock) 
	begin
		if(!reset_n)
			currentstate <= p1Move;
		else
			currentstate <= nextstate;
	end
		
	always @(*)
	begin
	select = 1'b0;
	p2Start = 1'b0;
	vgaDelayReset2 = 1'b0;
	vgaDelayReset3 = 1'b0;
		case (currentstate)
		p1Move:
			begin
			select = 1'b0;
			vgaDelayReset2 = 1'b1;
			end
		p2Move:
			begin
			p2Start = 1'b1;
			select = 1'b1;
			vgaDelayReset3 = 1'b1;
			end
		endcase
	end
				  
	multiSnake p1(
		.clock(clock),
		.reset_n(& {reset_n, multiModeReset, ~isDead1, ~isDead2}),
		.initX(7'b0010011),
		.initY(7'b0010011),
		.selfColor(3'b011),
		.gameStartButton(gameStart),
		.left(left),
		.right(right),
		.up(up),
		.down(down),
		.xout(p1X),
		.yout(p1Y),
		.cout(p1C), 
		.plot(p1Plot),
		.headX(HeadXp1), 
		.headY(HeadYp1),
		.bodyX(BodyXp1),
		.bodyY(BodyYp1),
		.goPrintNext(goPrintSnake2),
		.snakeGoPrintScreen(goPrintSnake1),
		.scoreyloc(7'b0001111)
		);
		
	multiSnake p2(
		.clock(clock),
		.reset_n(& {reset_n, multiModeReset, ~isDead1, ~isDead2}),
		.initX(7'b0101011),
		.initY(7'b0101011),
		.selfColor(3'b010),
		.gameStartButton(p2Start),
		.left(p2left),
		.right(p2right),
		.up(p2up),
		.down(p2down),
		.xout(p2X),
		.yout(p2Y),
		.cout(p2C), 
		.plot(p2Plot),
		.headX(HeadXp2), 
		.headY(HeadYp2),
		.bodyX(BodyXp2),
		.bodyY(BodyYp2),
		.goPrintNext(goPrintSnake1),
		.snakeGoPrintScreen(goPrintSnake2),
		.scoreyloc(7'b0111111)
		);
		
	vgaDelayCounter2 vgaDelay2(
		.clock(clock), 
		.reset_n(vgaDelayReset2),
		.vgaDelayOut(vgaDelayOut2)
		);
		
	vgaDelayCounter2 vgaDelay3(
		.clock(clock), 
		.reset_n(vgaDelayReset3),
		.vgaDelayOut(vgaDelayOut3)
		);
		
		multiPlayerCollisionDetect p1Detect(HeadXp1, HeadYp1, BodyXp2, BodyYp2, isDead1);
		multiPlayerCollisionDetect p2Detect(HeadXp2, HeadYp2, BodyXp1, BodyYp1, isDead2);
		
		myMux2to1 x(p1X, p2X, select, multiOutX);
		myMux2to1 y(p1Y, p2Y, select, multiOutY);
		myMux2to1 c(p1C, p2C, select, multiOutC);
		myMux2to1 p(p1Plot, p2Plot, select, multiOutPlot);
		
		always @(posedge clock)
		begin
			if (!reset_n)
				wholePageMultiC <= 3'b100;
			else if (isDead1 == 1'b1)
				wholePageMultiC <= 3'b010;
			else if (isDead2 == 1'b1)
				wholePageMultiC <= 3'b011;
		end
		
	
endmodule

module collisionDetect(headX, headY, bodyX, bodyY, isDead);
	input [7:0] headX, headY, bodyX, bodyY;
	
	output isDead;
	
	assign isDead = (headX == 7'b0000000) | (headX == 7'b0111011) | (headY == 7'b0000000) | (headY == 7'b0111011);
endmodule

module multiPlayerCollisionDetect(headX, headY, bodyX, bodyY, isDead);
	input [7:0] headX, headY, bodyX, bodyY;
	
	output isDead;
	
	assign isDead = (headX == 7'b0000000) | (headX == 7'b0111011) | (headY == 7'b0000000) | (headY == 7'b0111011) |
						 (headX == bodyX & headY == bodyY);
endmodule

module border(clock, reset_n, cin, xout, yout, cout, borderEnable);
	input clock, reset_n, borderEnable;
	input [2:0] cin;
	
	output [7:0] xout;
	output [6:0] yout;
	output [2:0] cout;
	
	reg left_en, down_en, right_en, up_en;
	reg [7:0] loc_x;
	reg [6:0] loc_y;
	
	always @(posedge clock)
	begin
		if (reset_n == 1'b0)
			begin
			loc_x <= 8'b00000010;
			loc_y <= 7'b0000010;
			left_en <= 1'b1;
			down_en <= 1'b0;
			right_en <= 1'b0;
			up_en <= 1'b0;
			end
		else if (borderEnable)
		begin
		if (left_en) 
			begin
				if(loc_y == 7'b1110110)
					begin
						left_en <= 1'b0;
						down_en <= 1'b1;
					end
				else
					loc_y <= loc_y + 1'b1;
			end
		if (down_en) 
			begin
				if(loc_x == 8'b01110110)
					begin
						down_en <= 1'b0;
						right_en <= 1'b1;
					end
				else
					loc_x <= loc_x + 1'b1;
			end
		if (right_en) 
			begin
				if(loc_y == 7'b0000010)
					begin
						right_en <= 1'b0;
						up_en <= 1'b1;
					end
				else
					loc_y <= loc_y - 1'b1;
			end
		if (up_en) 
			begin
				if(loc_x == 8'b00000010)
					begin
						up_en <= 1'b0;
					end
				else
					loc_x <= loc_x - 1'b1;
			end
		end
   end
	
	assign xout = loc_x;
	assign yout = loc_y;
	
	assign cout = cin;
	
endmodule

module wholePagePrint(clock, reset_n, wholePageEnable, cin, xout, yout, cout);
	input clock, reset_n, wholePageEnable;
	input [2:0] cin;
	
	output [7:0] xout;
	output [6:0] yout;
	output [2:0] cout;
	
	reg [7:0] xx, yy;
	reg yenable;
	
	always @(posedge clock)
		begin
		if (!reset_n)
			begin
			xx <= 0;
			end
		else
			begin
			if (xx == 7'b1110111)
				begin
				xx <= 0;
				yenable <= 1;
				end
			else
				begin
				xx <= xx + 1;
				yenable <= 0;
				end
			end
		end
		
	always @(posedge clock)
		begin
		if (!reset_n)
			yy <= 0;
		else if (yenable)
			begin
			if (yy == 7'b1110111)
				yy <= 0;
			else
				yy <= yy + 1;
			end
		end
		
	assign xout = xx;
	assign yout = yy;
	assign cout = cin;
	
endmodule

module cleanP2Score(clock, reset_n, xout, yout, cout);
	input clock, reset_n;
	
	output [7:0] xout;
	output [6:0] yout;
	output [2:0] cout;
	
	reg [7:0] xx, yy;
	reg yenable;
	
	always @(posedge clock)
		begin
		if (!reset_n)
			begin
			xx <= 0;
			end
		else
			begin
			if (xx == 7'b1110111)
				begin
				xx <= 0;
				yenable <= 1;
				end
			else
				begin
				xx <= xx + 1;
				yenable <= 0;
				end
			end
		end
		
	always @(posedge clock)
		begin
		if (!reset_n)
			yy <= 0;
		else if (yenable)
			begin
			if (yy == 7'b1110111)
				yy <= 0;
			else
				yy <= yy + 1;
			end
		end
		
	assign xout = 8'b10000110 + xx;
	assign yout = 7'b0111111 + yy;
	assign cout = 3'b000;
	
endmodule

module snack(clock, reset_n, changeLocation, snackXOut, snackYOut, snackCOut, snackXCheck, snackYCheck);
	input clock, reset_n, changeLocation;
	
	output [6:0] snackXOut, snackYOut, snackXCheck, snackYCheck;
	output [2:0] snackCOut;
	
	wire [6:0] randomOutX, randomOutY, x, y;
	
	reg [1:0] xx, yy;
	reg yenable;
	
	always @(posedge clock)
		begin
		if (!reset_n)
			begin
			xx <= 0;
			end
		else
			begin
			if (xx == 2'b01)
				begin
				xx <= 0;
				yenable <= 1;
				end
			else
				begin
				xx <= xx + 1;
				yenable <= 0;
				end
			end
		end
		
	always @(posedge clock)
		begin
		if (!reset_n)
			yy <= 0;
		else if (yenable)
			begin
			if (yy == 2'b01)
				yy <= 0;
			else
				yy <= yy + 1;
			end
		end
		
	snackLocationGenerator xlocation(clock, reset_n, 7'b0000110, changeLocation, randomOutX);
	snackLocationGenerator ylocation(clock, reset_n, 7'b0100100, changeLocation, randomOutY);

	assign x = randomOutX + 7'b0001010;
	assign y = randomOutY + 7'b0001010;

	assign snackXCheck = x;
	assign snackYCheck = y;
	assign snackXOut = 2 * x + xx;
	assign snackYOut = 2 * y + yy;
	assign snackCOut = 3'b110;

endmodule

module snackLocationGenerator(clock, reset_n, initLocation, changeLocation, randomOut);
    input clock, reset_n, changeLocation;
	 input [4:0] initLocation;
	 
	 output reg [4:0] randomOut;

	always@(posedge clock)
	begin
		if(!reset_n)
			randomOut <= initLocation;
		else if (changeLocation == 1'b1)
			begin
            randomOut[0] <= randomOut[4];
            randomOut[1] <= randomOut[0] ^ randomOut[4];
            randomOut[2] <= randomOut[1];
            randomOut[3] <= randomOut[2] ^ randomOut[4];
            randomOut[4] <= randomOut[3] ^ randomOut[4];
			end
		else
			randomOut <= randomOut;        
		end
endmodule

module myMux6to1(a, b, c, d, e, f, select, out);
	input [7:0] a, b, c, d, e, f;
	input [2:0] select;
	output reg [7:0] out;
	
	always @(*)
	begin
		case (select)
		3'b000: out = a;
		3'b001: out = b;
		3'b010: out = c;
		3'b011: out = d;
		3'b100: out = e;
		3'b101: out = f;
		default: out = a;
		endcase
	end
endmodule

module myMux2to1(a, b, select, out);
	input [7:0] a, b;
	input select;
	output reg [7:0] out;
	
	always @(*)
	begin
		case (select)
		1'b0: out = a;
		1'b1: out = b;
		default: out = a;
		endcase
	end
endmodule

module vgaDelayCounter2(clock, reset_n, vgaDelayOut);
	input clock, reset_n;
	
	output reg [19:0] vgaDelayOut;
  
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			vgaDelayOut <= 100;
		else
		begin
			if (vgaDelayOut == 20'd0)
				vgaDelayOut <= 5;
		else
				vgaDelayOut <= vgaDelayOut - 1'b1;
		end
	end
endmodule
