module snake(clock, reset_n, initX, initY, selfColor, gameStartButton, left, right, up, down, xout, yout, cout, plot, headX, headY, bodyX, bodyY, goPrintSnack, snakeGoPrintScreen, snackX, snackY, changeLocation, scoreyloc);
	input clock, reset_n, gameStartButton, left, right, up, down, snakeGoPrintScreen;
	input [6:0] initX, initY, snackX, snackY, scoreyloc;
	input [2:0] selfColor;
	//input [1:0] dir;
	
	output [7:0] xout, yout, headX, headY, bodyX, bodyY;
	output [2:0] cout;
	output plot, goPrintSnack, changeLocation;

	wire [5:0] snakeBodyLength, bodyUnitNumber;
	wire xdirEnable, ydirEnable, xupdown, yupdown, moveEnable, lengthChangeEnable, lengthUpdown, scoreReset, scoreEnable;
	wire [7:0] score_x;
	wire [6:0] score_y;
	wire [2:0] score_c;
	
	snakeBodyControlPath c1(
		.clock(clock),
		.reset_n(reset_n),
		.gameStartButton(gameStartButton),
		.snakeBodyLength(snakeBodyLength),
		.left(left),
		.right(right),
		.up(up),
		.down(down),
		.xdirEnable(xdirEnable),
		.ydirEnable(ydirEnable),
		.xupdown(xupdown),
		.yupdown(yupdown),
		.bodyUnitOut(bodyUnitNumber),
		.moveEnable(moveEnable),
		.eraseItem(eraseItem),
		.plot(plot),
		.currentHeadX(headX), 
		.currentHeadY(headY), 
		.currentBodyX(bodyX), 
		.currentBodyY(bodyY), 
		.lengthChangeEnable(lengthChangeEnable), 
		.lengthUpdown(lengthUpdown),
		.scoreEnable(scoreEnable),
		.scoreReset(scoreReset),
		.goPrintSnack(goPrintSnack),
		.snakeGoPrintScreen(snakeGoPrintScreen),
		.snackX(snackX), 
		.snackY(snackY),
		.changeLocation(changeLocation)
		);
	
	snakeBodyDataPath d1(
		.clock(clock),
		.reset_n(reset_n),
		.initX(initX),
		.initY(initY),
		.selfColor(selfColor),
		.moveEnable(moveEnable),
		.xdirEnable(xdirEnable),
		.ydirEnable(ydirEnable),
		.xupdown(xupdown),
		.yupdown(yupdown),
		.bodyUnitIn(bodyUnitNumber),
		.currLength(snakeBodyLength),
		.eraseItem(eraseItem),
		.snakeXOut(xout),
		.snakeYOut(yout),
		.snakeCOut(cout),
		.headXOut(headX), 
		.headYOut(headY),
		.bodyXOut(bodyX), 
		.bodyYOut(bodyY),
		.lengthChangeEnable(lengthChangeEnable), 
		.lengthUpdown(lengthUpdown),
		.scoreEnable(scoreEnable),
		.score_x(score_x),
		.score_y(score_y),
		.score_c(score_c),
		.initLength(6'b000111)
		);

	score s1(
		.clock(clock),
		.reset_n(reset_n),
		.scoreReset(scoreReset),
		.scoreEnable(scoreEnable),
		.scorein(snakeBodyLength),
		.cin(selfColor),
		.xout(score_x),
		.yout(score_y), 
		.cout(score_c),
		.scoreyloc(scoreyloc)
		);
		
endmodule

module multiSnake(clock, reset_n, initX, initY, selfColor, gameStartButton, left, right, up, down, xout, yout, cout, plot, headX, headY, bodyX, bodyY, goPrintNext, snakeGoPrintScreen, changeLocation, scoreyloc);
	input clock, reset_n, gameStartButton, left, right, up, down, snakeGoPrintScreen;
	input [6:0] initX, initY, scoreyloc;
	input [2:0] selfColor;
	//input [1:0] dir;
	
	output [7:0] xout, yout, headX, headY, bodyX, bodyY;
	output [2:0] cout;
	output plot, goPrintNext, changeLocation;

	wire [5:0] snakeBodyLength, bodyUnitNumber;
	wire xdirEnable, ydirEnable, xupdown, yupdown, moveEnable, lengthChangeEnable, lengthUpdown, scoreReset, scoreEnable;
	wire [7:0] score_x;
	wire [6:0] score_y;
	wire [2:0] score_c;
	
	multiSnakeBodyControlPath mc1(
		.clock(clock),
		.reset_n(reset_n),
		.gameStartButton(gameStartButton),
		.snakeBodyLength(snakeBodyLength),
		.left(left),
		.right(right),
		.up(up),
		.down(down),
		.xdirEnable(xdirEnable),
		.ydirEnable(ydirEnable),
		.xupdown(xupdown),
		.yupdown(yupdown),
		.bodyUnitOut(bodyUnitNumber),
		.moveEnable(moveEnable),
		.eraseItem(eraseItem),
		.plot(plot),
		.currentHeadX(headX), 
		.currentHeadY(headY), 
		.currentBodyX(bodyX), 
		.currentBodyY(bodyY), 
		.lengthChangeEnable(lengthChangeEnable), 
		.lengthUpdown(lengthUpdown),
		.scoreEnable(scoreEnable),
		.scoreReset(scoreReset),
		.goPrintNext(goPrintNext),
		.snakeGoPrintScreen(snakeGoPrintScreen),
		);
	
	snakeBodyDataPath md1(
		.clock(clock),
		.reset_n(reset_n),
		.initX(initX),
		.initY(initY),
		.selfColor(selfColor),
		.moveEnable(moveEnable),
		.xdirEnable(xdirEnable),
		.ydirEnable(ydirEnable),
		.xupdown(xupdown),
		.yupdown(yupdown),
		.bodyUnitIn(bodyUnitNumber),
		.currLength(snakeBodyLength),
		.eraseItem(eraseItem),
		.snakeXOut(xout),
		.snakeYOut(yout),
		.snakeCOut(cout),
		.headXOut(headX), 
		.headYOut(headY),
		.bodyXOut(bodyX), 
		.bodyYOut(bodyY),
		.lengthChangeEnable(lengthChangeEnable), 
		.lengthUpdown(lengthUpdown),
		.scoreEnable(scoreEnable),
		.score_x(score_x),
		.score_y(score_y),
		.score_c(score_c),
		.initLength(6'b101110)
		);

	score ms1(
		.clock(clock),
		.reset_n(reset_n),
		.scoreReset(scoreReset),
		.scoreEnable(scoreEnable),
		.scorein(snakeBodyLength),
		.cin(selfColor),
		.xout(score_x),
		.yout(score_y), 
		.cout(score_c),
		.scoreyloc(scoreyloc)
		);
		
endmodule
/*
Maximum snake body length = 30;
Clock should not be CLOCK_50, user rate divider to slow the clock;
*/
module snakeBodyDataPath(clock, reset_n, initX, initY, selfColor, moveEnable, xdirEnable, ydirEnable, xupdown, yupdown, bodyUnitIn, currLength, eraseItem, snakeXOut, snakeYOut, snakeCOut, headXOut, headYOut, bodyXOut, bodyYOut, lengthChangeEnable, lengthUpdown, scoreEnable, score_x, score_y, score_c, initLength);
	input clock, reset_n, xdirEnable, ydirEnable, xupdown, yupdown, eraseItem, moveEnable, lengthChangeEnable, lengthUpdown, scoreEnable;
	input [7:0] score_x;
	input [6:0] score_y;
	input [2:0] score_c;
	input [6:0] initX, initY;
	input [5:0] bodyUnitIn, initLength;
	input [2:0] selfColor;
	
	wire [6:0] currentHeadX, currentHeadY;
	wire [6:0] x, y;
	
	output [5:0] currLength;
	output[7:0] snakeXOut, snakeYOut;
	output [2:0] snakeCOut;
	
	output [7:0] headXOut, headYOut, bodyXOut, bodyYOut;
	
	bodyLengthCounter b0(
		.clock(clock), 
		.reset_n(reset_n), 
		.bodyLengthCounterEnable(lengthChangeEnable), 
		.bodyLengthOut(currLength),
		.updown(lengthUpdown),
		.initLength(initLength)
		);
			
	
	headLocationCounter headX(
		.rateDividedClock(clock),
		.reset_n(reset_n),
		.moveEnable(xdirEnable & moveEnable),
		.updown(xupdown),
		.initLocation(initX),
		.headLocationOut(currentHeadX)
		);

	snakeBodyShifter bodyX(
		.clock(clock),
		.reset_n(reset_n),
		.shiftEnable(moveEnable),
		.rightMost(currentHeadX),
		.shifterSelect(bodyUnitIn), 
		.shifterOut(x)
		);
	
	headLocationCounter headY(
		.rateDividedClock(clock),
		.reset_n(reset_n),
		.moveEnable(ydirEnable & moveEnable),
		.updown(yupdown),
		.initLocation(initY),
		.headLocationOut(currentHeadY)
		);
		
	snakeBodyShifter bodyY(
		.clock(clock),
		.reset_n(reset_n),
		.shiftEnable(moveEnable),
		.rightMost(currentHeadY),
		.shifterSelect(bodyUnitIn), 
		.shifterOut(y)
		);


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

		
	assign snakeXOut = (scoreEnable) ? score_x : 2 * x + xx;
	assign snakeYOut = (scoreEnable) ? score_y : 2 * y + yy;
	
	assign headXOut = currentHeadX;
	assign headYOut = currentHeadY;
	
	assign bodyXOut = x;
	assign bodyYOut = y;

	
	assign snakeCOut = eraseItem ? 3'b000 : ((scoreEnable) ? score_c : selfColor);
	
endmodule

module snakeBodyControlPath(clock, reset_n, gameStartButton, snakeBodyLength, left, right, up, down, xdirEnable, ydirEnable, xupdown, yupdown, bodyUnitOut, moveEnable, eraseItem, plot, currentHeadX, currentHeadY, currentBodyX, currentBodyY, lengthChangeEnable, lengthUpdown, scoreEnable, scoreReset, goPrintSnack, snakeGoPrintScreen, snackX, snackY, changeLocation);
	input clock, reset_n, gameStartButton, left, right, up, down, snakeGoPrintScreen;
	input [7:0] currentHeadX, currentHeadY, currentBodyX, currentBodyY, snackX, snackY;
	input [5:0] snakeBodyLength;
	
	reg gameSpeedReset, vgaDelayReset, bodyUnitReset, updateLocationReset, eraseTail, vgaDelayReset1;
	wire goLengthDecrease, goLengthIncrease;
	reg [3:0]currentstate, nextstate;
	//reg [1:0]currentdir, nextdir;
	
	wire [27:0] gameSpeedDelay;
	wire [19:0] vgaDelayOut, vgaDelayOut1;
	wire [27:0] updateLocationOut;
	
	reg [1:0] dir; //00: left, 01: right, 10: up, 11: down
	
	output [5:0] bodyUnitOut;
	output reg plot, xdirEnable, ydirEnable, xupdown, yupdown, moveEnable, eraseItem, lengthChangeEnable, lengthUpdown, scoreEnable, scoreReset, goPrintSnack, changeLocation;
	
	localparam init 				    = 4'b0000,
				  printScreen         = 4'b0001,
				  erase               = 4'b0010,
				  erase_wait          = 4'b0011,
				  selfCollisionDetect = 4'b0100,
				  lengthDecrease      = 4'b0101,
				  eraseLast           = 4'b0110,
				  lengthIncrease      = 4'b0111,
				  updateLocation      = 4'b1000,
				  updateScore         = 4'b1001,
				  printSnackState     = 4'b1010;
				  
	localparam leftMove  = 2'd0,
				  rightMove = 2'd1,
				  upMove    = 2'd2,
				  downMove  = 2'd3;
				  
	assign goLengthDecrease = (currentHeadX == currentBodyX & currentHeadY == currentBodyY);
	assign goLengthIncrease = (currentHeadX == snackX & currentHeadY == snackY);
	
	always @(*)
	begin
		case (currentstate)
			init: nextstate = gameStartButton ? printScreen : init;
			printScreen: nextstate = (gameSpeedDelay == 0) ? erase : printScreen;
			erase: nextstate =  (vgaDelayOut == 0) ? erase_wait : erase;
			erase_wait: nextstate = selfCollisionDetect;
			selfCollisionDetect: 
				begin
				if (goLengthDecrease == 1'b1)
					nextstate = lengthDecrease;
				else if (goLengthIncrease == 1'b1)
					nextstate = lengthIncrease;
				else if (bodyUnitOut == snakeBodyLength)
					nextstate = updateLocation;
				else
					nextstate = selfCollisionDetect;
				end
			lengthDecrease: nextstate = eraseLast;
			eraseLast: nextstate = (vgaDelayOut == 0) ? (goLengthIncrease ? lengthIncrease : updateLocation) : eraseLast;
			lengthIncrease: nextstate = updateLocation;
			updateLocation: nextstate = moveEnable ? updateScore : updateLocation;
			updateScore: nextstate = (vgaDelayOut1 == 0) ? printSnackState : updateScore;
			printSnackState: nextstate = snakeGoPrintScreen ? printScreen : printSnackState;
			default: nextstate = init;
		endcase
	end
	

	always @(posedge clock)
		begin
		if (reset_n == 1'b0)
			dir <= rightMove;
		else if (left == 1'b1 & dir != rightMove)
			dir <= leftMove;
		else if (right == 1'b1 & dir != leftMove)
			dir <= rightMove;
		else if (up == 1'b1 & dir != downMove)
			dir <= upMove;
		else if (down == 1'b1 & dir != upMove)
			dir <= downMove;
		end
	
	always @(*)
	begin
		plot = 1'b0;
		gameSpeedReset = 1'b0;
		vgaDelayReset = 1'b0;
		vgaDelayReset1 = 1'b0;
		bodyUnitReset = 1'b0;
		updateLocationReset = 1'b0;
		moveEnable = 1'b0;
		eraseItem = 1'b0;
		eraseTail = 1'b0;
		lengthChangeEnable = 1'b0;
		lengthUpdown = 1'b0;
		scoreReset = 1'b0;
		scoreEnable = 1'b0;
		goPrintSnack = 1'b0;
		changeLocation = 1'b0;
		case (currentstate)
			printScreen:
				begin
				gameSpeedReset = 1'b1;
				bodyUnitReset = 1'b1;
				plot = 1'b1;
				end
			erase:
				begin
				eraseItem = 1'b1;
				gameSpeedReset = 1'b0;
				bodyUnitReset = 1'b1;
				vgaDelayReset = 1'b1;
				plot = 1'b1;
				end
			erase_wait:
				begin
				bodyUnitReset = 1'b0;
				end
			selfCollisionDetect:
				begin
				bodyUnitReset = 1'b1;
				end
			lengthDecrease:
				begin
				bodyUnitReset = 1'b0;
				lengthChangeEnable = 1'b1;
				lengthUpdown = 1'b0;
				end
			eraseLast:
				begin
				bodyUnitReset = 1'b1;
				eraseTail = 1'b1;
				plot = 1'b1;
				eraseItem = 1'b1;
				vgaDelayReset = 1'b1;
				end
			lengthIncrease:
				begin
				lengthChangeEnable = 1'b1;
				lengthUpdown = 1'b1;
				changeLocation = 1'b1;
				end
			updateLocation:
				begin
				updateLocationReset = 1'b0;
				moveEnable = 1'b1;
				changeLocation = 1'b0;
				//moveEnable = (updateLocationOut == 28'b01011111010111100000111110110) ? 1'b1 : 1'b0;
				end
			updateScore:
				begin
				scoreEnable = 1'b1;
				vgaDelayReset1 = 1'b1;
				plot = 1'b1;
				scoreReset = 1'b1;
				end
			printSnackState:
				begin
				goPrintSnack = 1'b1;
				end
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
		xdirEnable = 1'b1;
		ydirEnable = 1'b0;
		xupdown = 1'b1;
		yupdown = 1'b0;
		case (dir)
			leftMove: begin //left move
				xdirEnable = 1'b1;
				ydirEnable = 1'b0;
				xupdown = 1'b0;
				yupdown = 1'b0;
				end
			rightMove: begin //right move
				xdirEnable = 1'b1;
				xupdown = 1'b1;
				yupdown = 1'b0;
				ydirEnable = 1'b0;
				end
			upMove: begin //up move
				xdirEnable = 1'b0;
				ydirEnable = 1'b1;
				yupdown = 1'b0;
				end
			downMove: begin //down move
				xdirEnable = 1'b0;
				ydirEnable = 1'b1;
				yupdown = 1'b1;
				end
		endcase
	end

	
	updateLocationCounter update(
		.clock(clock), 
		.reset_n(updateLocationReset), 
		.updateLocationCounterOut(updateLocationOut)
		);
	
	bodyUnitOutputCounter bodyUnit(
		.clock(clock), 
		.reset_n(bodyUnitReset), 
		.bodyLength(snakeBodyLength), 
		.bodyUnitOut(bodyUnitOut),
		.eraseTail(eraseTail)
		);
	
	gameSpeedMux gameSpeedControl(
		.clock(clock), 
		.reset_n(gameSpeedReset), 
		.currentLength(snakeBodyLength), 
		.speedOut(gameSpeedDelay)
		);
		
	vgaDelayCounter vgaDelay(
		.clock(clock), 
		.reset_n(vgaDelayReset),
		.vgaDelayOut(vgaDelayOut)
		);
		
	vgaDelayCounter1 vgaDelay1(
		.clock(clock), 
		.reset_n(vgaDelayReset1),
		.vgaDelayOut(vgaDelayOut1)
		);
endmodule

module multiSnakeBodyControlPath(clock, reset_n, gameStartButton, snakeBodyLength, left, right, up, down, xdirEnable, ydirEnable, xupdown, yupdown, bodyUnitOut, moveEnable, eraseItem, plot, currentHeadX, currentHeadY, currentBodyX, currentBodyY, lengthChangeEnable, lengthUpdown, scoreEnable, scoreReset, goPrintNext, snakeGoPrintScreen);
	input clock, reset_n, gameStartButton, left, right, up, down, snakeGoPrintScreen;
	input [7:0] currentHeadX, currentHeadY, currentBodyX, currentBodyY;
	input [5:0] snakeBodyLength;
	
	reg gameSpeedReset, vgaDelayReset, bodyUnitReset, updateLocationReset, eraseTail, vgaDelayReset1;
	wire goLengthDecrease;
	reg [3:0]currentstate, nextstate;
	//reg [1:0]currentdir, nextdir;
	
	wire [27:0] gameSpeedDelay;
	wire [19:0] vgaDelayOut, vgaDelayOut1;
	wire [27:0] updateLocationOut;
	
	reg [1:0] dir; //00: left, 01: right, 10: up, 11: down
	
	output [5:0] bodyUnitOut;
	output reg plot, xdirEnable, ydirEnable, xupdown, yupdown, moveEnable, eraseItem, lengthChangeEnable, lengthUpdown, scoreEnable, scoreReset, goPrintNext;
	
	localparam init 				    = 4'b0000,
				  printScreen         = 4'b0001,
				  erase               = 4'b0010,
				  erase_wait          = 4'b0011,
				  selfCollisionDetect = 4'b0100,
				  lengthDecrease      = 4'b0101,
				  eraseLast           = 4'b0110,
				  updateLocation      = 4'b1000,
				  updateScore         = 4'b1001,
				  printNextSnake      = 4'b1010,
				  printNextWait       = 4'b1011;
				  
	localparam leftMove  = 2'd0,
				  rightMove = 2'd1,
				  upMove    = 2'd2,
				  downMove  = 2'd3;
				  
	assign goLengthDecrease = (currentHeadX == currentBodyX & currentHeadY == currentBodyY);
	
	always @(*)
	begin
		case (currentstate)
			init: nextstate = gameStartButton ? printScreen : init;
			printScreen: nextstate = (gameSpeedDelay == 0) ? erase : printScreen;
			erase: nextstate =  (vgaDelayOut == 0) ? erase_wait : erase;
			erase_wait: nextstate = selfCollisionDetect;
			selfCollisionDetect: 
				begin
				if (goLengthDecrease == 1'b1)
					nextstate = lengthDecrease;
				else if (bodyUnitOut == snakeBodyLength)
					nextstate = updateLocation;
				else
					nextstate = selfCollisionDetect;
				end
			lengthDecrease: nextstate = eraseLast;
			eraseLast: nextstate = (vgaDelayOut == 0) ? updateLocation : eraseLast;
			updateLocation: nextstate = moveEnable ? updateScore : updateLocation;
			updateScore: nextstate = (vgaDelayOut1 == 0) ? printNextSnake : updateScore;
			printNextSnake: nextstate = printNextWait;
			printNextWait: nextstate = snakeGoPrintScreen ? printScreen : printNextWait;
			default: nextstate = init;
		endcase
	end
	

	always @(posedge clock)
		begin
		if (reset_n == 1'b0)
			dir <= rightMove;
		else if (left == 1'b1 & dir != rightMove)
			dir <= leftMove;
		else if (right == 1'b1 & dir != leftMove)
			dir <= rightMove;
		else if (up == 1'b1 & dir != downMove)
			dir <= upMove;
		else if (down == 1'b1 & dir != upMove)
			dir <= downMove;
		end
	
	always @(*)
	begin
		plot = 1'b0;
		gameSpeedReset = 1'b0;
		vgaDelayReset = 1'b0;
		vgaDelayReset1 = 1'b0;
		bodyUnitReset = 1'b0;
		updateLocationReset = 1'b0;
		moveEnable = 1'b0;
		eraseItem = 1'b0;
		eraseTail = 1'b0;
		lengthChangeEnable = 1'b0;
		lengthUpdown = 1'b0;
		scoreReset = 1'b0;
		scoreEnable = 1'b0;
		goPrintNext = 1'b0;
		case (currentstate)
			printScreen:
				begin
				gameSpeedReset = 1'b1;
				bodyUnitReset = 1'b1;
				plot = 1'b1;
				end
			erase:
				begin
				eraseItem = 1'b1;
				gameSpeedReset = 1'b0;
				bodyUnitReset = 1'b1;
				vgaDelayReset = 1'b1;
				plot = 1'b1;
				end
			erase_wait:
				begin
				bodyUnitReset = 1'b0;
				end
			selfCollisionDetect:
				begin
				bodyUnitReset = 1'b1;
				end
			lengthDecrease:
				begin
				bodyUnitReset = 1'b0;
				lengthChangeEnable = 1'b1;
				lengthUpdown = 1'b0;
				end
			eraseLast:
				begin
				bodyUnitReset = 1'b1;
				eraseTail = 1'b1;
				plot = 1'b1;
				eraseItem = 1'b1;
				vgaDelayReset = 1'b1;
				end
			updateLocation:
				begin
				updateLocationReset = 1'b0;
				moveEnable = 1'b1;
				//moveEnable = (updateLocationOut == 28'b01011111010111100000111110110) ? 1'b1 : 1'b0;
				end
			updateScore:
				begin
				scoreEnable = 1'b1;
				vgaDelayReset1 = 1'b1;
				plot = 1'b1;
				scoreReset = 1'b1;
				end
			printNextSnake:
				begin
				goPrintNext = 1'b1;
				end
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
		xdirEnable = 1'b1;
		ydirEnable = 1'b0;
		xupdown = 1'b1;
		yupdown = 1'b0;
		case (dir)
			leftMove: begin //left move
				xdirEnable = 1'b1;
				ydirEnable = 1'b0;
				xupdown = 1'b0;
				yupdown = 1'b0;
				end
			rightMove: begin //right move
				xdirEnable = 1'b1;
				xupdown = 1'b1;
				yupdown = 1'b0;
				ydirEnable = 1'b0;
				end
			upMove: begin //up move
				xdirEnable = 1'b0;
				ydirEnable = 1'b1;
				yupdown = 1'b0;
				end
			downMove: begin //down move
				xdirEnable = 1'b0;
				ydirEnable = 1'b1;
				yupdown = 1'b1;
				end
		endcase
	end

	
	updateLocationCounter mupdate(
		.clock(clock), 
		.reset_n(updateLocationReset), 
		.updateLocationCounterOut(updateLocationOut)
		);
	
	bodyUnitOutputCounter mbodyUnit(
		.clock(clock), 
		.reset_n(bodyUnitReset), 
		.bodyLength(snakeBodyLength), 
		.bodyUnitOut(bodyUnitOut),
		.eraseTail(eraseTail)
		);
	
	
	gameSpeedCounter mtwentyTwoHzC(
		.clock(clock),
		.reset_n(gameSpeedReset),
		.gameSpeed(28'b0000001000101010110111010111),
		.gameSpeedCounterOut(gameSpeedDelay)
	
	);
	
	/*
	gameSpeedMux mgameSpeedControl(
		.clock(clock), 
		.reset_n(gameSpeedReset), 
		.currentLength(snakeBodyLength), 
		.speedOut(gameSpeedDelay)
		);
	*/
		
	vgaDelayCounter mvgaDelay(
		.clock(clock), 
		.reset_n(vgaDelayReset),
		.vgaDelayOut(vgaDelayOut)
		);
		
	vgaDelayCounter1 mvgaDelay1(
		.clock(clock), 
		.reset_n(vgaDelayReset1),
		.vgaDelayOut(vgaDelayOut1)
		);
endmodule

module gameSpeedMux(clock, reset_n, currentLength, speedOut);
	input clock, reset_n;
	input [5:0] currentLength;
	
	output reg [27:0] speedOut;
	
	wire [27:0] twoHz, fourHz, sixHz, eightHz, tenHz, twelveHz, twentyTwoHz;
	
	gameSpeedCounter twoHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0001011111010111100000111111),
		.gameSpeedCounterOut(twoHz)
		);
		
	gameSpeedCounter fourHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0000101111101011110000011111),
		.gameSpeedCounterOut(fourHz)
		);

	gameSpeedCounter sixHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0000011111110010100000010101),
		.gameSpeedCounterOut(sixHz)
		);

	gameSpeedCounter eightHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0000010111110101111000001111),
		.gameSpeedCounterOut(eightHz)
		);
		
	gameSpeedCounter tenHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0000010011000100101101000000),
		.gameSpeedCounterOut(tenHz)
		);
		
	gameSpeedCounter twelveHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0000001111111001010000001010),
		.gameSpeedCounterOut(twelveHz)
		);
		
	gameSpeedCounter twentyTwoHzC(
		.clock(clock),
		.reset_n(reset_n),
		.gameSpeed(28'b0000001000101010110111010111),
		.gameSpeedCounterOut(twentyTwoHz)
		);

	always @(*)
	begin
		case (currentLength)
			6'b000000: speedOut = twoHz;
			6'b000001: speedOut = twoHz;
			6'b000010: speedOut = twoHz;
			6'b000011: speedOut = twoHz;
			6'b000100: speedOut = twoHz;
			6'b000101: speedOut = twoHz;
			6'b000110: speedOut = twoHz;
			6'b000111: speedOut = twoHz;
			6'b001000: speedOut = twoHz;
			6'b001001: speedOut = twoHz;
			6'b001010: speedOut = fourHz;
			6'b001011: speedOut = fourHz;
			6'b001100: speedOut = fourHz;
			6'b001101: speedOut = fourHz;
			6'b001110: speedOut = fourHz;
			6'b001111: speedOut = fourHz;
			6'b010000: speedOut = fourHz;
			6'b010001: speedOut = fourHz;
			6'b010010: speedOut = fourHz;
			6'b010011: speedOut = fourHz;
			6'b010100: speedOut = sixHz;
			6'b010101: speedOut = sixHz;
			6'b010110: speedOut = sixHz;
			6'b010111: speedOut = sixHz;
			6'b011000: speedOut = sixHz;
			6'b011001: speedOut = sixHz;
			6'b011010: speedOut = sixHz;
			6'b011011: speedOut = sixHz;
			6'b011100: speedOut = sixHz;
			6'b011101: speedOut = sixHz;
			6'b011110: speedOut = eightHz;
			6'b011111: speedOut = eightHz;
			6'b100000: speedOut = eightHz;
			6'b100001: speedOut = eightHz;
			6'b100010: speedOut = eightHz;
			6'b100011: speedOut = eightHz;
			6'b100100: speedOut = eightHz;
			6'b100101: speedOut = eightHz;
			6'b100110: speedOut = eightHz;
			6'b100111: speedOut = eightHz;
			6'b101000: speedOut = tenHz;
			6'b101001: speedOut = tenHz;
			6'b101010: speedOut = tenHz;
			6'b101011: speedOut = tenHz;
			6'b101100: speedOut = tenHz;
			6'b101101: speedOut = tenHz;
			6'b101110: speedOut = tenHz;
			6'b101111: speedOut = tenHz;
			6'b110000: speedOut = tenHz;
			6'b110001: speedOut = tenHz;
			6'b110010: speedOut = twelveHz;
			6'b110011: speedOut = twelveHz;
			6'b110100: speedOut = twelveHz;
			6'b110101: speedOut = twelveHz;
			6'b110110: speedOut = twelveHz;
			6'b110111: speedOut = twelveHz;
			6'b111000: speedOut = twentyTwoHz;
			6'b111001: speedOut = twentyTwoHz;
			6'b111010: speedOut = twentyTwoHz;
			6'b111011: speedOut = twentyTwoHz;
		endcase
	end

endmodule


module headLocationCounter(rateDividedClock, reset_n, moveEnable, updown, initLocation, headLocationOut);
	input rateDividedClock, reset_n, moveEnable, updown;
	input [6:0] initLocation;
	
	output reg [6:0] headLocationOut;
	
	always @(posedge rateDividedClock)
	begin
		if (reset_n == 1'b0)
			headLocationOut <= initLocation;
		else if (moveEnable == 1'b1)
		begin
			if (updown == 1'b1)
			begin
				if (headLocationOut == 7'b0111011)
					headLocationOut <= initLocation;
				else
					headLocationOut <= headLocationOut + 1;
			end
			else if (updown == 1'b0)
			begin
				if (headLocationOut == 7'b0000000)
					headLocationOut <= initLocation;
				else
					headLocationOut <= headLocationOut - 1;
			end
		end
	end
	
endmodule

/*
rightMost connect with headLocationCounter to store new location after move.
*/
module snakeBodyShifter(clock, reset_n, shiftEnable, rightMost, shifterSelect, shifterOut);
	input shiftEnable, clock, reset_n;
	input [6:0] rightMost;
	input [5:0] shifterSelect;
	
	output reg [6:0] shifterOut;

	wire [6:0] q00, q01, q02, q03, q04, q05, q06, q07, q08, q09, q10, q11, q12, q13, q14, q15, q16, q17, q18, q19;
	wire [6:0] q20, q21, q22, q23, q24, q25, q26, q27, q28, q29, q30, q31, q32, q33, q34, q35, q36, q37, q38, q39;
	wire [6:0] q40, q41, q42, q43, q44, q45, q46, q47, q48, q49, q50, q51, q52, q53, q54, q55, q56, q57, q58, q59;

	shifterBit7bits s59(.reset_n(reset_n), .clock(clock), .in(q58), .shift(shiftEnable), .out(q59));
	shifterBit7bits s58(.reset_n(reset_n), .clock(clock), .in(q57), .shift(shiftEnable), .out(q58));
	shifterBit7bits s57(.reset_n(reset_n), .clock(clock), .in(q56), .shift(shiftEnable), .out(q57));
	shifterBit7bits s56(.reset_n(reset_n), .clock(clock), .in(q55), .shift(shiftEnable), .out(q56));
	shifterBit7bits s55(.reset_n(reset_n), .clock(clock), .in(q54), .shift(shiftEnable), .out(q55));
	shifterBit7bits s54(.reset_n(reset_n), .clock(clock), .in(q53), .shift(shiftEnable), .out(q54));
	shifterBit7bits s53(.reset_n(reset_n), .clock(clock), .in(q52), .shift(shiftEnable), .out(q53));
	shifterBit7bits s52(.reset_n(reset_n), .clock(clock), .in(q51), .shift(shiftEnable), .out(q52));
	shifterBit7bits s51(.reset_n(reset_n), .clock(clock), .in(q50), .shift(shiftEnable), .out(q51));
	shifterBit7bits s50(.reset_n(reset_n), .clock(clock), .in(q49), .shift(shiftEnable), .out(q50));
	shifterBit7bits s49(.reset_n(reset_n), .clock(clock), .in(q48), .shift(shiftEnable), .out(q49));
	shifterBit7bits s48(.reset_n(reset_n), .clock(clock), .in(q47), .shift(shiftEnable), .out(q48));
	shifterBit7bits s47(.reset_n(reset_n), .clock(clock), .in(q46), .shift(shiftEnable), .out(q47));
	shifterBit7bits s46(.reset_n(reset_n), .clock(clock), .in(q45), .shift(shiftEnable), .out(q46));
	shifterBit7bits s45(.reset_n(reset_n), .clock(clock), .in(q44), .shift(shiftEnable), .out(q45));
	shifterBit7bits s44(.reset_n(reset_n), .clock(clock), .in(q43), .shift(shiftEnable), .out(q44));
	shifterBit7bits s43(.reset_n(reset_n), .clock(clock), .in(q42), .shift(shiftEnable), .out(q43));
	shifterBit7bits s42(.reset_n(reset_n), .clock(clock), .in(q41), .shift(shiftEnable), .out(q42));
	shifterBit7bits s41(.reset_n(reset_n), .clock(clock), .in(q40), .shift(shiftEnable), .out(q41));
	shifterBit7bits s40(.reset_n(reset_n), .clock(clock), .in(q39), .shift(shiftEnable), .out(q40));
	shifterBit7bits s39(.reset_n(reset_n), .clock(clock), .in(q38), .shift(shiftEnable), .out(q39));
	shifterBit7bits s38(.reset_n(reset_n), .clock(clock), .in(q37), .shift(shiftEnable), .out(q38));
	shifterBit7bits s37(.reset_n(reset_n), .clock(clock), .in(q36), .shift(shiftEnable), .out(q37));
	shifterBit7bits s36(.reset_n(reset_n), .clock(clock), .in(q35), .shift(shiftEnable), .out(q36));
	shifterBit7bits s35(.reset_n(reset_n), .clock(clock), .in(q34), .shift(shiftEnable), .out(q35));
	shifterBit7bits s34(.reset_n(reset_n), .clock(clock), .in(q33), .shift(shiftEnable), .out(q34));
	shifterBit7bits s33(.reset_n(reset_n), .clock(clock), .in(q32), .shift(shiftEnable), .out(q33));
	shifterBit7bits s32(.reset_n(reset_n), .clock(clock), .in(q31), .shift(shiftEnable), .out(q32));
	shifterBit7bits s31(.reset_n(reset_n), .clock(clock), .in(q30), .shift(shiftEnable), .out(q31));
	shifterBit7bits s30(.reset_n(reset_n), .clock(clock), .in(q29), .shift(shiftEnable), .out(q30));
	shifterBit7bits s29(.reset_n(reset_n), .clock(clock), .in(q28), .shift(shiftEnable), .out(q29));
	shifterBit7bits s28(.reset_n(reset_n), .clock(clock), .in(q27), .shift(shiftEnable), .out(q28));
	shifterBit7bits s27(.reset_n(reset_n), .clock(clock), .in(q26), .shift(shiftEnable), .out(q27));
	shifterBit7bits s26(.reset_n(reset_n), .clock(clock), .in(q25), .shift(shiftEnable), .out(q26));
	shifterBit7bits s25(.reset_n(reset_n), .clock(clock), .in(q24), .shift(shiftEnable), .out(q25));
	shifterBit7bits s24(.reset_n(reset_n), .clock(clock), .in(q23), .shift(shiftEnable), .out(q24));
	shifterBit7bits s23(.reset_n(reset_n), .clock(clock), .in(q22), .shift(shiftEnable), .out(q23));
	shifterBit7bits s22(.reset_n(reset_n), .clock(clock), .in(q21), .shift(shiftEnable), .out(q22));
	shifterBit7bits s21(.reset_n(reset_n), .clock(clock), .in(q20), .shift(shiftEnable), .out(q21));
	shifterBit7bits s20(.reset_n(reset_n), .clock(clock), .in(q19), .shift(shiftEnable), .out(q20));
	shifterBit7bits s19(.reset_n(reset_n), .clock(clock), .in(q18), .shift(shiftEnable), .out(q19));
	shifterBit7bits s18(.reset_n(reset_n), .clock(clock), .in(q17), .shift(shiftEnable), .out(q18));
	shifterBit7bits s17(.reset_n(reset_n), .clock(clock), .in(q16), .shift(shiftEnable), .out(q17));
	shifterBit7bits s16(.reset_n(reset_n), .clock(clock), .in(q15), .shift(shiftEnable), .out(q16));
	shifterBit7bits s15(.reset_n(reset_n), .clock(clock), .in(q14), .shift(shiftEnable), .out(q15));
	shifterBit7bits s14(.reset_n(reset_n), .clock(clock), .in(q13), .shift(shiftEnable), .out(q14));
	shifterBit7bits s13(.reset_n(reset_n), .clock(clock), .in(q12), .shift(shiftEnable), .out(q13));
	shifterBit7bits s12(.reset_n(reset_n), .clock(clock), .in(q11), .shift(shiftEnable), .out(q12));
	shifterBit7bits s11(.reset_n(reset_n), .clock(clock), .in(q10), .shift(shiftEnable), .out(q11));
	shifterBit7bits s10(.reset_n(reset_n), .clock(clock), .in(q09), .shift(shiftEnable), .out(q10));
	shifterBit7bits s09(.reset_n(reset_n), .clock(clock), .in(q08), .shift(shiftEnable), .out(q09));
	shifterBit7bits s08(.reset_n(reset_n), .clock(clock), .in(q07), .shift(shiftEnable), .out(q08));
	shifterBit7bits s07(.reset_n(reset_n), .clock(clock), .in(q06), .shift(shiftEnable), .out(q07));
	shifterBit7bits s06(.reset_n(reset_n), .clock(clock), .in(q05), .shift(shiftEnable), .out(q06));
	shifterBit7bits s05(.reset_n(reset_n), .clock(clock), .in(q04), .shift(shiftEnable), .out(q05));
	shifterBit7bits s04(.reset_n(reset_n), .clock(clock), .in(q03), .shift(shiftEnable), .out(q04));
	shifterBit7bits s03(.reset_n(reset_n), .clock(clock), .in(q02), .shift(shiftEnable), .out(q03));
	shifterBit7bits s02(.reset_n(reset_n), .clock(clock), .in(q01), .shift(shiftEnable), .out(q02));
	shifterBit7bits s01(.reset_n(reset_n), .clock(clock), .in(q00), .shift(shiftEnable), .out(q01));
	shifterBit7bits s00(.reset_n(reset_n), .clock(clock), .in(rightMost), .shift(shiftEnable), .out(q00));
	
	always @(*)
	begin
		case (shifterSelect)
			6'b000000: shifterOut = q00;
			6'b000001: shifterOut = q01;
			6'b000010: shifterOut = q02;
			6'b000011: shifterOut = q03;
			6'b000100: shifterOut = q04;
			6'b000101: shifterOut = q05;
			6'b000110: shifterOut = q06;
			6'b000111: shifterOut = q07;
			6'b001000: shifterOut = q08;
			6'b001001: shifterOut = q09;
			6'b001010: shifterOut = q10;
			6'b001011: shifterOut = q11;
			6'b001100: shifterOut = q12;
			6'b001101: shifterOut = q13;
			6'b001110: shifterOut = q14;
			6'b001111: shifterOut = q15;
			6'b010000: shifterOut = q16;
			6'b010001: shifterOut = q17;
			6'b010010: shifterOut = q18;
			6'b010011: shifterOut = q19;
			6'b010100: shifterOut = q20;
			6'b010101: shifterOut = q21;
			6'b010110: shifterOut = q22;
			6'b010111: shifterOut = q23;
			6'b011000: shifterOut = q24;
			6'b011001: shifterOut = q25;
			6'b011010: shifterOut = q26;
			6'b011011: shifterOut = q27;
			6'b011100: shifterOut = q28;
			6'b011101: shifterOut = q29;
			6'b011110: shifterOut = q30;
			6'b011111: shifterOut = q31;
			6'b100000: shifterOut = q32;
			6'b100001: shifterOut = q33;
			6'b100010: shifterOut = q34;
			6'b100011: shifterOut = q35;
			6'b100100: shifterOut = q36;
			6'b100101: shifterOut = q37;
			6'b100110: shifterOut = q38;
			6'b100111: shifterOut = q39;
			6'b101000: shifterOut = q40;
			6'b101001: shifterOut = q41;
			6'b101010: shifterOut = q42;
			6'b101011: shifterOut = q43;
			6'b101100: shifterOut = q44;
			6'b101101: shifterOut = q45;
			6'b101110: shifterOut = q46;
			6'b101111: shifterOut = q47;
			6'b110000: shifterOut = q48;
			6'b110001: shifterOut = q49;
			6'b110010: shifterOut = q50;
			6'b110011: shifterOut = q51;
			6'b110100: shifterOut = q52;
			6'b110101: shifterOut = q53;
			6'b110110: shifterOut = q54;
			6'b110111: shifterOut = q55;
			6'b111000: shifterOut = q56;
			6'b111001: shifterOut = q57;
			6'b111010: shifterOut = q58;
			6'b111011: shifterOut = q59;
		endcase
	end

endmodule

module bodyLengthCounter(clock, reset_n, bodyLengthCounterEnable, bodyLengthOut, updown, initLength);
	input clock, reset_n, bodyLengthCounterEnable, updown;
	input [5:0] initLength;
	
	output reg [5:0] bodyLengthOut;
	
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			bodyLengthOut <= initLength;
		else if(bodyLengthCounterEnable == 1'b1 & updown ==  1'b1)
		begin
			if(bodyLengthOut == 6'b111000)
				bodyLengthOut <= 6'b111000;
			else
				bodyLengthOut <= bodyLengthOut + 1'b1;
		end
		else if(bodyLengthCounterEnable == 1'b1 & updown ==  1'b0)
		begin
			if(bodyLengthOut == 6'b000000)
				bodyLengthOut <= 6'b000000;
			else
				bodyLengthOut <= bodyLengthOut - 1'b1;
		end
	end
endmodule

module bodyUnitOutputCounter(clock, reset_n, bodyLength, bodyUnitOut, eraseTail);
	input clock, reset_n, eraseTail;
	input [5:0]bodyLength;
	
	wire [3:0] bodyUnitDelayOut;
	
	output reg [5:0] bodyUnitOut;

	always @(posedge clock)
		begin
		if (reset_n == 1'b0)
			bodyUnitOut <= 6'b000000;
		else if (eraseTail == 1'b1)
			bodyUnitOut <= (bodyLength);
		else if (bodyUnitDelayOut == 4'b0000 & eraseTail == 1'b0)
			begin
			if (bodyUnitOut == bodyLength)
				bodyUnitOut <= bodyLength;
			else
				bodyUnitOut <= bodyUnitOut + 1;
			end
		end

		bodyUnitOutputDelayCounter b0(clock, reset_n, bodyUnitDelayOut);
		
endmodule

module bodyUnitOutputDelayCounter(clock, reset_n, bodyUnitDelayOut);
	input clock, reset_n;
	
	output reg [3:0] bodyUnitDelayOut;
	
	always @(posedge clock)
		begin
		if (reset_n == 1'b0)
			bodyUnitDelayOut <= 4'b1010;
		else
			begin
			if (bodyUnitDelayOut == 4'b0000)
				bodyUnitDelayOut <= 4'b1010;
			else
				bodyUnitDelayOut <= bodyUnitDelayOut - 1;
			end
		end

endmodule

/*
Game speed counter;
1Hz: 28'b0010111110101111000001111111
2Hz: 28'b0001011111010111100000111111
3hz: 28'b0000111111100101000000101010
4Hz: 28'b0000101111101011110000011111
5hz: 28'b0000100110001001011010000000
6hz: 28'b0000011111110010100000010101
*/
module gameSpeedCounter(clock, reset_n, gameSpeed, gameSpeedCounterOut);
	input clock, reset_n;
	input [27:0] gameSpeed;
	output reg [27:0] gameSpeedCounterOut;
	
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			gameSpeedCounterOut <= gameSpeed;
		else
			begin
				if (gameSpeedCounterOut == 28'b0000000000000000000000000000)
					gameSpeedCounterOut <= gameSpeed;
				else
					gameSpeedCounterOut <= gameSpeedCounterOut - 1'b1;
			end
	end
endmodule

module updateLocationCounter(clock, reset_n, updateLocationCounterOut);
	input clock, reset_n;
	output reg [27:0] updateLocationCounterOut;
	
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			updateLocationCounterOut <= 28'b0101111101011110000011111111;
		else
			begin
				if (updateLocationCounterOut == 28'b0000000000000000000000000000)
					updateLocationCounterOut <= 28'b0101111101011110000011111111;
				else
					updateLocationCounterOut <= updateLocationCounterOut - 1'b1;
			end
	end
endmodule

module vgaDelayCounter(clock, reset_n, vgaDelayOut);
	input clock, reset_n;
	
	output reg [19:0] vgaDelayOut;
  
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			vgaDelayOut <= 833334;
		else
		begin
			if (vgaDelayOut == 20'd0)
				vgaDelayOut <= 833334;
		else
				vgaDelayOut <= vgaDelayOut - 1'b1;
		end
	end
	
endmodule

module vgaDelayCounter1(clock, reset_n, vgaDelayOut);
	input clock, reset_n;
	
	output reg [19:0] vgaDelayOut;
  
	always @(posedge clock)
	begin
		if(reset_n == 1'b0)
			vgaDelayOut <= 100;
		else
		begin
			if (vgaDelayOut == 20'd0)
				vgaDelayOut <= 100;
		else
				vgaDelayOut <= vgaDelayOut - 1'b1;
		end
	end
	
endmodule

module shifterBit7bits(reset_n, clock, in, shift, out);
	input [6:0] in;
	input shift, clock, reset_n;
	
	output [6:0] out;
	
	wire [6:0] w1, out;
	
	mux2to1 M1(
		.x(out),
		.y(in),
		.s(shift),
		.m(w1)
	);

	register7bits r0(
		.d(w1),
		.out(out),
		.clock(clock),
		.reset_n(reset_n)
	);

endmodule

module mux2to1(x, y, s, m);
    input [6:0] x; //selected when s is 0
    input [6:0] y; //selected when s is 1
    input s; //select signal
    output reg [6:0] m; //output
  
    always @(*)
	 begin
		case (s)
			1'b0: m = x;
			1'b1: m = y;
		endcase
	end
		
    // OR
    // assign m = s ? y : x;

endmodule

module register7bits(d, out, clock, reset_n);
	input clock;
	input reset_n;
	input [6:0] d;
	
	output reg [6:0] out;
	
	always @(posedge clock)
	begin
		if (reset_n == 1'b0)
			out <= 7'b0000000;
		else
			out <= d;
	end
endmodule
/*
module score(clock, reset_n, scoreReset, scoreEnable, xout, yout, cout);
	input clock, reset_n, scoreReset, scoreEnable;
	
	output [7:0] xout;
	output [6:0] yout;
	output [2:0] cout;
	
	wire [7:0] xb;
	wire [6:0] yb;
	wire [2:0] cb;
	
	number n0(
				.xout(xb), 
				.yout(yb), 
				.cout(cb), 
				.num(3'd3), 
				.xin(8'b10000000), 
				.yin(7'b0001111), 
				.cin(3'b111), 
				.clk(CLOCK_50), 
				.reset_n(scoreReset),
				.scoreEnable(scoreEnable)
				);
				
	assign xout = xb;
	assign yout = yb;
	assign cout = cb;
endmodule
*/

module score(clock, reset_n, scoreReset, scoreEnable, scorein, cin, xout, yout, cout, scoreyloc);
	input clock, reset_n, scoreReset, scoreEnable;
	input [9:0] scorein;
	input [2:0] cin;
	input [6:0] scoreyloc;
	
	output [7:0] xout;
	output [6:0] yout;
	output [2:0] cout;
	
	reg [3:0] count;
	reg [1:0] index;
	reg [7:0] xout;
	reg [6:0] yout;
	reg [2:0] cout;
	wire [7:0] xg;
	wire [6:0] yg;
	wire [2:0] cg;
	wire [7:0] xs;
	wire [6:0] ys;
	wire [2:0] cs;
	wire [7:0] xb;
	wire [6:0] yb;
	wire [2:0] cb;
	
	wire [4:0] score_g;
	wire [4:0] score_s;
	wire [4:0] score_b;
	
	assign score_g = scorein % 4'b1010;
	assign score_s = scorein / 4'b1010;
	
	always @(posedge clock)
		begin
			if (scoreReset == 1'b0)
				count <= 0;
			else
				begin
				if (scoreEnable)
					begin
					if (count == 4'b1111)
						count <= 0;
					else
						count <= count + 1'b1;
					end
				end
		end

	always @(posedge clock)
		begin
			if (scoreReset == 1'b0)
				index <= 0;
			else 
				begin
				if(count == 4'b1111 & scoreEnable)
					begin
					if (index == 2'b10)
						index <= 0;
					else
						index <= index + 1'b1;
					end
				end
		end		

	number n0(
				.xout(xb), 
				.yout(yb), 
				.cout(cb), 
				.num(3'b000), 
				.xin(8'b10000110), 
				.yin(scoreyloc), 
				.cin(cin), 
				.clk(clock), 
				.reset_n(scoreReset),
				.scoreEnable(scoreEnable)
				);
				
	number n1(
				.xout(xs), 
				.yout(ys), 
				.cout(cs), 
				.num(score_s), 
				.xin(8'b10001010), 
				.yin(scoreyloc), 
				.cin(cin), 
				.clk(clock), 
				.reset_n(scoreReset),
				.scoreEnable(scoreEnable)
				);
				
	number n2(
				.xout(xg), 
				.yout(yg), 
				.cout(cg), 
				.num(score_g), 
				.xin(8'b10001110), 
				.yin(scoreyloc), 
				.cin(cin), 
				.clk(clock), 
				.reset_n(scoreReset),
				.scoreEnable(scoreEnable)
				);
				
			
	always @(*)
		begin
			case(index)
				0: begin
						xout <= xb;
						yout <= yb;
						cout <= cb;
					end
				1: begin
						xout <= xs;
						yout <= ys;
						cout <= cs;
					end
				2: begin
						xout <= xg;
						yout <= yg;
						cout <= cg;
					end
			endcase
		end
		
endmodule

module number(xout, yout, cout, num, xin, yin, cin, clk, reset_n, scoreEnable);
	input [7:0] xin;
	input [6:0] yin;
	input [2:0] cin;
	input [3:0] num;
	input clk, reset_n, scoreEnable;
	output [7:0] xout;
	output [6:0] yout;
	output [2:0] cout;
	wire [14:0] list;
	
	hex h0(
			.list(list), 
			.num(num)
			);
			
	single s0(
				.xout(xout), 
				.yout(yout), 
				.cout(cout), 
				.xin(xin), 
				.yin(yin), 
				.cin(cin), 
				.clk(clk), 
				.reset_n(reset_n), 
				.scoreEnable(scoreEnable),
				.list(list)
				);
endmodule


module hex(list, num);
	input [3:0] num;
	output reg [14:0] list;
	
	always @(*)
		begin
			case(num)
				0: list <= 15'b111101101101111;
				1: list <= 15'b010010010010010;
				2: list <= 15'b111001111100111;
				3: list <= 15'b111001111001111;
				4: list <= 15'b101101111001001;
				5: list <= 15'b111100111001111;
				6: list <= 15'b111100111101111;
				7: list <= 15'b111001001001001;
				8: list <= 15'b111101111101111;
				9: list <= 15'b111101111001111;
				default: list <= 15'b000000000000000;
			endcase
		end
endmodule
	
module single(xout, yout, cout, xin, yin, cin, clk, reset_n, scoreEnable, list);
	input [7:0] xin;
	input [6:0] yin;
	input [2:0] cin;
	input [0:14] list;
	input clk, reset_n, scoreEnable;
	output [7:0] xout;
	output [6:0] yout;
	output reg [2:0] cout;
	wire enable_y;
	reg [1:0] xindex;
	reg [2:0] yindex;
	reg [3:0] counter;
	
	always @(posedge clk)
		begin
			if (reset_n == 1'b0)
				counter <= 0;
			else
				begin
				if (scoreEnable)
					begin
					if (counter == 4'b1110)
						counter <= 0;
					else
						counter <= counter + 1'b1;
				   end
				end
		end
	
	always @(posedge clk)
		begin
			if (reset_n == 1'b0)
				xindex <= 0;
			else 
				begin
				if (scoreEnable)
					begin
					if (xindex == 2'b10)
						xindex <= 0;
					else
						xindex <= xindex + 1'b1;
					end
				end
		end
	assign xout = xin + xindex;
	
	assign enable_y = (xindex == 2'b10) ? 1'b1 : 1'b0;
	
	always @(posedge clk)
		begin
			if (reset_n == 1'b0)
				yindex <= 0;
			else 
				begin
				if (enable_y == 1'b1 & scoreEnable)
				begin
					if (yindex == 3'b100)
						yindex <= 0;
					else
						yindex <= yindex + 1'b1;
					end
				end
		end
	assign yout = yin + yindex;
	
	always @(*)
		begin
			if (reset_n == 1'b0 | list[counter] == 1'b0)
				cout[2:0] = 3'b000;
			else
				cout[2:0] = cin;
		end
endmodule
