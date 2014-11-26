LIBRARY IEEE;
LIBRARY ALTERA_MF;
LIBRARY LPM;

USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.STD_LOGIC_ARITH.ALL;
USE IEEE.STD_LOGIC_UNSIGNED.ALL;
USE ALTERA_MF.ALTERA_MF_COMPONENTS.ALL;
USE LPM.LPM_COMPONENTS.ALL;


ENTITY SCOMP IS
	PORT(
		CLOCK    : IN    STD_LOGIC;
		RESETN   : IN    STD_LOGIC;
		PCINT    : IN    STD_LOGIC_VECTOR( 3 DOWNTO 0);
		IO_WRITE : OUT   STD_LOGIC;
		IO_CYCLE : OUT   STD_LOGIC;
		IO_ADDR  : OUT   STD_LOGIC_VECTOR( 7 DOWNTO 0);
		IO_DATA  : INOUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END SCOMP;


ARCHITECTURE a OF SCOMP IS
	TYPE STATE_TYPE IS (
		RESET_PC,
		FETCH,
		DECODE,
		EX_LOAD,
		EX_STORE,
		EX_STORE2,
		EX_ADD,
		EX_SUB,
		EX_JUMP,
		EX_JNEG,
		EX_JPOS,
		EX_JZERO,
		EX_AND,
		EX_OR,
		EX_XOR,
		EX_SHIFT,
		EX_ADDI,
		EX_ILOAD,
		EX_ISTORE,
		EX_CALL,
		EX_RETURN,
		EX_IN,
		EX_OUT,
		EX_OUT2,
		EX_LOADI,
		EX_RETI,
		EX_LOADA,
		EX_SWITCH,
		EX_SWITCH2,
		EX_COPY,
		EX_ADDA,
		EX_ADDIA,
		EX_STOREA,
		EX_STOREA2,
		EX_MULTA,
		EX_OUTA,
		EX_OUTA2,
		EX_OUTA3,
		EX_INA,
		EX_SUBA,
		EX_LOC,
		EX_ANDI
	);

	TYPE STACK_TYPE IS ARRAY (0 TO 7) OF STD_LOGIC_VECTOR(9 DOWNTO 0);
	TYPE AC_ARRAY IS ARRAY (0 TO 7) OF STD_LOGIC_VECTOR(15 DOWNTO 0);
	TYPE BIGGER_STACK IS ARRAY (0 TO 11) OF STD_LOGIC_VECTOR(9 DOWNTO 0);

	SIGNAL STATE        : STATE_TYPE;
	SIGNAL PC_STACK     : BIGGER_STACK;
	SIGNAL IO_IN        : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL AC           : AC_ARRAY;
	SIGNAL AC_SAVED     : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL AC_SHIFTED   : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL IR           : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL MDR          : STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL PC           : STD_LOGIC_VECTOR( 9 DOWNTO 0);
	SIGNAL PC_SAVED     : STD_LOGIC_VECTOR( 9 DOWNTO 0);
	SIGNAL MEM_ADDR     : STD_LOGIC_VECTOR( 9 DOWNTO 0);
	SIGNAL MW           : STD_LOGIC;
	SIGNAL IO_WRITE_INT : STD_LOGIC;
	SIGNAL GIE          : STD_LOGIC;
	SIGNAL IIE     		: STD_LOGIC_VECTOR( 3 DOWNTO 0);
	SIGNAL INT_REQ      : STD_LOGIC_VECTOR( 3 DOWNTO 0);
	SIGNAL INT_REQ_SYNC : STD_LOGIC_VECTOR( 3 DOWNTO 0); -- registered version of INT_REQ
	SIGNAL INT_ACK      : STD_LOGIC_VECTOR( 3 DOWNTO 0);
	SIGNAL IN_HOLD      : STD_LOGIC;
	SIGNAL AC_TEMP		: STD_LOGIC_VECTOR(15 DOWNTO 0);
	SIGNAL AC_TEMP2		: STD_LOGIC_VECTOR(15 DOWNTO 0);


BEGIN
	-- Use altsyncram component for unified program and data memory
	MEMORY : altsyncram
	GENERIC MAP (
		intended_device_family => "Cyclone",
		width_a          => 16,
		widthad_a        => 10,
		numwords_a       => 1024,
		operation_mode   => "SINGLE_PORT",
		outdata_reg_a    => "UNREGISTERED",
		indata_aclr_a    => "NONE",
		wrcontrol_aclr_a => "NONE",
		address_aclr_a   => "NONE",
		outdata_aclr_a   => "NONE",
		init_file        => "albertPathPlanning.mif",
		lpm_hint         => "ENABLE_RUNTIME_MOD=NO",
		lpm_type         => "altsyncram"
	)
	PORT MAP (
		wren_a    => MW,
		clock0    => NOT(CLOCK),
		address_a => MEM_ADDR,
		data_a    => AC(0),
		q_a       => MDR
	);

	-- Use LPM function to shift AC using the SHIFT instruction
	SHIFTER: LPM_CLSHIFT
	GENERIC MAP (
		lpm_width     => 16,
		lpm_widthdist => 4,
		lpm_shifttype => "ARITHMETIC"
	)
	PORT MAP (
		data      => AC(0),
		distance  => IR(3 DOWNTO 0),
		direction => IR(4),
		result    => AC_SHIFTED
	);

	-- Use LPM function to drive I/O bus
	IO_BUS: LPM_BUSTRI
	GENERIC MAP (
		lpm_width => 16
	)
	PORT MAP (
		data     => AC(0),
		enabledt => IO_WRITE_INT,
		tridata  => IO_DATA
	);


	IO_ADDR  <= IR(7 DOWNTO 0);

	WITH STATE SELECT MEM_ADDR <=
		PC WHEN FETCH,
		AC(conv_integer(IR(4 DOWNTO 0)))(9 DOWNTO 0) WHEN EX_STOREA,
		AC(conv_integer(IR(4 DOWNTO 0)))(9 DOWNTO 0) WHEN EX_STOREA2,
		AC(conv_integer(IR(4 DOWNTO 0)))(9 DOWNTO 0) WHEN EX_LOADA,
		IR(9 DOWNTO 0) WHEN OTHERS;

	WITH STATE SELECT IO_CYCLE <=
		'1' WHEN EX_IN,
		'1' WHEN EX_INA,
		'1' WHEN EX_OUT2,
		'1' WHEN EX_OUTA3,
		'0' WHEN OTHERS;

	IO_WRITE <= IO_WRITE_INT;

	PROCESS (CLOCK, RESETN)
	BEGIN
		IF (RESETN = '0') THEN          -- Active low, asynchronous reset
			STATE <= RESET_PC;
		ELSIF (RISING_EDGE(CLOCK)) THEN
			CASE STATE IS
				WHEN RESET_PC =>
					MW        <= '0';          -- Clear memory write flag
					PC        <= "0000000000"; -- Reset PC to the beginning of memory, address 0x000
					AC(0)        <= x"0000";      -- Clear AC register
					FOR i IN 0 TO 7 LOOP
						AC(i) <= x"0000";
					END LOOP;
					IO_WRITE_INT <= '0';
					GIE       <= '1';          -- Enable interrupts
					IIE       <= "0000";       -- Mask all interrupts
					STATE     <= FETCH;
					IN_HOLD   <= '0';
					INT_REQ_SYNC <= "0000";

				WHEN FETCH =>
					MW    <= '0';       -- Clear memory write flag
					IR    <= MDR;       -- Latch instruction into the IR
					IO_WRITE_INT <= '0';       -- Lower IO_WRITE after an OUT
					-- Interrupt Control
					IF (GIE = '1') AND  -- If Global Interrupt Enable set and...
					  (INT_REQ_SYNC /= "0000") THEN -- ...an interrupt is pending
						IF INT_REQ_SYNC(0) = '1' THEN   -- Got interrupt on PCINT0
							INT_ACK <= "0001";     -- Acknowledge the interrupt
							PC <= "0000000001";    -- Redirect execution
						ELSIF INT_REQ_SYNC(1) = '1' THEN
							INT_ACK <= "0010";     -- repeat for other pins
							PC <= "0000000010";
						ELSIF INT_REQ_SYNC(2) = '1' THEN
							INT_ACK <= "0100";
							PC <= "0000000011";
						ELSIF INT_REQ_SYNC(3) = '1' THEN
							INT_ACK <= "1000";
							PC <= "0000000100";
						END IF;
						GIE <= '0';            -- Disable interrupts while in ISR
						AC_SAVED <= AC(0);        -- Save AC
						PC_SAVED <= PC;        -- Save PC
						STATE <= FETCH;        -- Repeat FETCH with new PC
					ELSE -- either no interrupt or interrupts disabled
						PC        <= PC + 1;   -- Increment PC to next instruction address
						STATE     <= DECODE;
						INT_ACK   <= "0000";   -- Clear any interrupt acknowledge
					END IF;

				WHEN DECODE =>
					CASE IR(15 downto 10) IS
						WHEN "000000" =>       -- No Operation (NOP)
							STATE <= FETCH;
						WHEN "000001" =>       -- LOAD
							STATE <= EX_LOAD;
						WHEN "000010" =>       -- STORE
							STATE <= EX_STORE;
						WHEN "000011" =>       -- ADD
							STATE <= EX_ADD;
						WHEN "000100" =>       -- SUB
							STATE <= EX_SUB;
						WHEN "000101" =>       -- JUMP
							STATE <= EX_JUMP;
						WHEN "000110" =>       -- JNEG
							STATE <= EX_JNEG;
						WHEN "000111" =>       -- JPOS
							STATE <= EX_JPOS;
						WHEN "001000" =>       -- JZERO
							STATE <= EX_JZERO;
						WHEN "001001" =>       -- AND
							STATE <= EX_AND;
						WHEN "001010" =>       -- OR
							STATE <= EX_OR;
						WHEN "001011" =>       -- XOR
							STATE <= EX_XOR;
						WHEN "001100" =>       -- SHIFT
							STATE <= EX_SHIFT;
						WHEN "001101" =>       -- ADDI
							STATE <= EX_ADDI;
						WHEN "001110" =>       -- ILOAD
							STATE <= EX_ILOAD;
						WHEN "001111" =>       -- ISTORE
							STATE <= EX_ISTORE;
						WHEN "010000" =>       -- CALL
							STATE <= EX_CALL;
						WHEN "010001" =>       -- RETURN
							STATE <= EX_RETURN;
						WHEN "010010" =>       -- IN
							STATE <= EX_IN;
						WHEN "010011" =>       -- OUT
							IO_WRITE_INT <= '1'; -- raise IO_WRITE
							STATE <= EX_OUT;
						WHEN "010100" =>       -- CLI
							IIE <= IIE AND NOT(IR(3 DOWNTO 0));  -- disable indicated interrupts
							STATE <= FETCH;
						WHEN "010101" =>       -- SEI
							IIE <= IIE OR IR(3 DOWNTO 0);  -- enable indicated interrupts
							STATE <= FETCH;
						WHEN "010110" =>       -- RETI
							STATE <= EX_RETI;
						WHEN "010111" =>       -- LOADI
							STATE <= EX_LOADI;
						WHEN "01"&x"8" =>	   -- LOADA
							AC_TEMP <= IR;
							STATE <= EX_LOADA;
						WHEN "01"&x"9" =>	   -- SWITCH
							STATE <= EX_SWITCH;
						WHEN "10"&x"1" =>		-- COPY
							STATE <= EX_COPY;
						WHEN "10"&x"2" =>		-- ADDA
							STATE <= EX_ADDA;
						WHEN "10"&x"3" =>		-- ADDIA
							STATE <= EX_ADDIA;
						WHEN "10"&x"4" =>		-- STOREA
							STATE <= EX_STOREA;
						WHEN "10"&x"5" =>		-- MULTA
							STATE <= EX_MULTA;
						WHEN "10"&x"6" =>		-- OUTA
							STATE <= EX_OUTA;
						WHEN "10"&x"7" =>
							STATE <= EX_INA;
						WHEN "10"&x"8" =>
							STATE <= EX_SUBA;	
						WHEN "10"&x"9" =>
							STATE <= EX_LOC;
						WHEN "10"&x"A" =>
							STATE <= EX_ANDI;

						WHEN OTHERS =>
							STATE <= FETCH;      -- Invalid opcodes default to NOP
					END CASE;

				WHEN EX_LOAD =>
					AC(0)    <= MDR;            -- Latch data from MDR (memory contents) to AC
					STATE <= FETCH;

				WHEN EX_STORE =>
					MW    <= '1';            -- Raise MW to write AC to MEM
					STATE <= EX_STORE2;

				WHEN EX_STORE2 =>
					MW    <= '0';            -- Drop MW to end write cycle
					STATE <= FETCH;

				WHEN EX_ADD =>
					AC(0)    <= AC(0) + MDR;
					STATE <= FETCH;

				WHEN EX_SUB =>
					AC(0)    <= AC(0) - MDR;
					STATE <= FETCH;

				WHEN EX_JUMP =>
					PC    <= IR(9 DOWNTO 0);
					STATE <= FETCH;

				WHEN EX_JNEG =>
					IF (AC(0)(15) = '1') THEN
						PC    <= IR(9 DOWNTO 0);
					END IF;

				STATE <= FETCH;

				WHEN EX_JPOS =>
					IF ((AC(0)(15) = '0') AND (AC(0) /= x"0000")) THEN
						PC    <= IR(9 DOWNTO 0);
					END IF;
					STATE <= FETCH;

				WHEN EX_JZERO =>
					IF (AC(0) = x"0000") THEN
						PC    <= IR(9 DOWNTO 0);
					END IF;
					STATE <= FETCH;

				WHEN EX_AND =>
					AC(0)    <= AC(0) AND MDR;
					STATE <= FETCH;

				WHEN EX_OR =>
					AC(0)    <= AC(0) OR MDR;
					STATE <= FETCH;

				WHEN EX_XOR =>
					AC(0)    <= AC(0) XOR MDR;
					STATE <= FETCH;

				WHEN EX_SHIFT =>
					AC(0)    <= AC_SHIFTED;
					STATE <= FETCH;

				WHEN EX_ADDI =>
					AC(0)    <= AC(0) + (IR(9) & IR(9) & IR(9) & IR(9) &
					 IR(9) & IR(9) & IR(9 DOWNTO 0));
					STATE <= FETCH;

				WHEN EX_ILOAD =>
					IR(9 DOWNTO 0) <= MDR(9 DOWNTO 0);
					STATE <= EX_LOAD;

				WHEN EX_ISTORE =>
					IR(9 DOWNTO 0) <= MDR(9 DOWNTO 0);
					STATE          <= EX_STORE;

				WHEN EX_CALL =>
					FOR i IN 0 TO 10 LOOP
						PC_STACK(i + 1) <= PC_STACK(i);
					END LOOP;
					PC_STACK(0) <= PC;
					PC          <= IR(9 DOWNTO 0);
					STATE       <= FETCH;

				WHEN EX_RETURN =>
					FOR i IN 0 TO 10 LOOP
						PC_STACK(i) <= PC_STACK(i + 1);
					END LOOP;
					PC          <= PC_STACK(0);
					STATE       <= FETCH;

				WHEN EX_IN =>
					IF IN_HOLD = '0' THEN
						AC(0)    <= IO_DATA;
						IN_HOLD <= '1';
					ELSE
						STATE <= FETCH;
						IN_HOLD <= '0';
					END IF;

				WHEN EX_OUT =>
					STATE <= EX_OUT2;

				WHEN EX_OUT2 =>
					IO_WRITE_INT <= '0';
					STATE <= FETCH;

				WHEN EX_LOADI =>
					AC(0)    <= (IR(9) & IR(9) & IR(9) & IR(9) &
					 IR(9) & IR(9) & IR(9 DOWNTO 0));
					STATE <= FETCH;

				WHEN EX_RETI =>
					GIE   <= '1';      -- re-enable interrupts
					PC    <= PC_SAVED; -- restore saved registers
					AC(0)    <= AC_SAVED;
					STATE <= FETCH;
				
				WHEN EX_LOADA =>		-- MDR = MEM(IR + AC)
					IR(9 DOWNTO 0) <= AC(conv_integer(AC_TEMP(4 DOWNTO 0)))(9 DOWNTO 0);
					AC(conv_integer(AC_TEMP(9 DOWNTO 5))) <= MDR;
					--AC(conv_integer(AC_TEMP(9 DOWNTO 5))) <= x"0000";
					STATE <= FETCH;
					
					
				WHEN EX_SWITCH =>
					AC_TEMP <= AC(conv_integer(IR(5 DOWNTO 3)));
					AC_TEMP2 <= AC(conv_integer(IR(2 DOWNTO 0)));
					STATE <= EX_SWITCH2;
				
				WHEN EX_SWITCH2 =>
					AC(conv_integer(IR(2 DOWNTO 0))) <= AC_TEMP;
					AC(conv_integer(IR(5 DOWNTO 3))) <= AC_TEMP2;
					STATE <= FETCH;
				
				WHEN EX_COPY =>
					AC(conv_integer(IR(9 DOWNTO 5))) <= AC(conv_integer(IR(4 DOWNTO 0)));
					STATE <= FETCH; 
				
				WHEN EX_ADDA =>	
					AC(conv_integer(IR(9 DOWNTO 6))) <= AC(conv_integer(IR(5 DOWNTO 3))) + AC(conv_integer(IR(2 DOWNTO 0)));
					STATE <= FETCH;
					
				WHEN EX_ADDIA =>
					AC(conv_integer(IR(9 DOWNTO 7))) <= AC(conv_integer(IR(9 DOWNTO 7))) + (IR(6)&IR(6)&IR(6)&IR(6)&IR(6)&IR(6)&IR(6)&IR(6)&IR(6)&IR(6 DOWNTO 0));
					STATE <= FETCH;
					
				WHEN EX_STOREA =>
					AC_TEMP <= AC(0);
					AC(0) <= AC(conv_integer(IR(9 DOWNTO 5)));
					MW <= '1';
					STATE <= EX_STOREA2;
				
				WHEN EX_STOREA2 =>
					MW <= '0';
					AC(0) <= AC_TEMP;
					STATE <= FETCH;
					
				WHEN EX_MULTA =>
					AC(conv_integer(IR(9 DOWNTO 7))) <= signed(AC(conv_integer(IR(6 DOWNTO 4)))) * signed(AC(conv_integer(IR(3 DOWNTO 0))));
					STATE <= FETCH;
					
				WHEN EX_OUTA =>
					AC_TEMP <= AC(0);
					AC(0) <= AC(conv_integer(IR(9 DOWNTO 8)));
					--AC(0)	<= x"3333";
					STATE <= EX_OUTA2;
					IO_WRITE_INT <= '1'; -- raise IO_WRITE

				WHEN EX_OUTA2 =>
					STATE <= EX_OUTA3;
					
				WHEN EX_OUTA3 =>
					IO_WRITE_INT <= '0'; -- raise IO_WRITE
					AC(0) <= AC_TEMP;
					STATE <= FETCH;
				
				WHEN EX_INA =>
					IF IN_HOLD = '0' THEN
						AC(conv_integer(IR(9 DOWNTO 8)))    <= IO_DATA;
						IN_HOLD <= '1';
					ELSE
						STATE <= FETCH;
						IN_HOLD <= '0';
					END IF;
				
				WHEN EX_SUBA =>
					AC(conv_integer(IR(9 DOWNTO 7))) <= AC(conv_integer(IR(6 DOWNTO 4))) - AC(conv_integer(IR(3 DOWNTO 0)));
					STATE <= FETCH;
				
				WHEN EX_ANDI =>
					AC(0) <= AC(0) AND ("000000" & IR(9 DOWNTO 0));
					STATE <= FETCH;
				
				WHEN EX_LOC =>
					CASE AC(0)(15 DOWNTO 0) IS
						WHEN "0000000000000101" => 	AC(0) <= "0000001000110100";
						WHEN "0000000101000000" => 	AC(0) <= "0000001011110100";
						WHEN "0000000000101000" => 	AC(0) <= "0000001010110100";
						WHEN "0000101000000000" => 	AC(0) <= "0000001001110100";
						WHEN "0000000001001100" => 	AC(0) <= "0000010000101100";
						WHEN "0000001100001000" => 	AC(0) <= "0000010011101100";
						WHEN "0000001000100001" => 	AC(0) <= "0000010010101100";
						WHEN "0000100001000001" => 	AC(0) <= "0000010001101100";
						WHEN "0000000001010011" => 	AC(0) <= "0000011000100100";
						WHEN "0000010011001000" => 	AC(0) <= "0000011011100100";
						WHEN "0000001000011010" => 	AC(0) <= "0000011010100100";
						WHEN "0000011010000001" => 	AC(0) <= "0000011001100100";
						WHEN "0000000001011010" => 	AC(0) <= "0000100000011100";
						WHEN "0000011010001000" => 	AC(0) <= "0000100011011100";
						WHEN "0000001000010011" => 	AC(0) <= "0000100010011100";
						WHEN "0000010011000001" => 	AC(0) <= "0000100001011100";
						WHEN "0000000011100001" => 	AC(0) <= "0000101000010100";
						WHEN "0000100001011000" => 	AC(0) <= "0000101011010100";
						WHEN "0000011000001100" => 	AC(0) <= "0000101010010100";
						WHEN "0000001100000011" => 	AC(0) <= "0000101001010100";
						WHEN "0000000011101000" => 	AC(0) <= "0000110000001100";
						WHEN "0000101000011000" => 	AC(0) <= "0000110011001100";
						WHEN "0000011000000101" => 	AC(0) <= "0000110010001100";
						WHEN "0000000101000011" => 	AC(0) <= "0000110001001100";
						WHEN "0000001010100000" => 	AC(0) <= "0000111000001011";
						WHEN "0000100000010001" => 	AC(0) <= "0000111011001011";
						WHEN "0000010001000100" => 	AC(0) <= "0000111010001011";
						WHEN "0000000010001100" => 	AC(0) <= "0000111001001011";
						WHEN "0000010001011000" => 	AC(0) <= "0001000000001010";
						WHEN "0000011000001010" => 	AC(0) <= "0001000011001010";
						WHEN "0000001010000011" => 	AC(0) <= "0001000010001010";
						WHEN "0000000011010001" => 	AC(0) <= "0001000001001010";
						WHEN "0000011000011000" => 	AC(0) <= "0001001000001001";
						WHEN "0000011000000011" => 	AC(0) <= "0001001011001001";
						WHEN "0000000011000011" => 	AC(0) <= "0001001010001001";
						WHEN "0000000011011000" => 	AC(0) <= "0001001001001001";
						WHEN "0000011000010001" => 	AC(0) <= "0001010000010001";
						WHEN "0000010001000011" => 	AC(0) <= "0001010011010001";
						WHEN "0000000011001010" => 	AC(0) <= "0001010010010001";
						WHEN "0000001010011000" => 	AC(0) <= "0001010001010001";
						WHEN "0000001000001010" => 	AC(0) <= "0001011000011001";
						WHEN "0000001010000001" => 	AC(0) <= "0001011011011001";
						WHEN "0000000001010001" => 	AC(0) <= "0001011010011001";
						WHEN "0000010001001000" => 	AC(0) <= "0001011001011001";
						WHEN "0000001000000011" => 	AC(0) <= "0001100000100001";
						WHEN "0000000011000001" => 	AC(0) <= "0001100011100001";
						WHEN "0000000001011000" => 	AC(0) <= "0001100010100001";
						WHEN "0000011000001000" => 	AC(0) <= "0001100001100001";
						WHEN "0000000001000011" => 	AC(0) <= "0001101000100010";
						WHEN "0000000011001000" => 	AC(0) <= "0001101011100010";
						WHEN "0000001000011000" => 	AC(0) <= "0001101010100010";
						WHEN "0000011000000001" => 	AC(0) <= "0001101001100010";
						WHEN "0000000001001010" => 	AC(0) <= "0001110000011010";
						WHEN "0000001010001000" => 	AC(0) <= "0001110011011010";
						WHEN "0000001000010001" => 	AC(0) <= "0001110010011010";
						WHEN "0000010001000001" => 	AC(0) <= "0001110001011010";
						WHEN "0000010001010001" => 	AC(0) <= "0001111000010010";
						WHEN "0000010001001010" => 	AC(0) <= "0001111011010010";
						WHEN "0000001010001010" => 	AC(0) <= "0001111010010010";
						WHEN "0000001010010001" => 	AC(0) <= "0001111001010010";
						WHEN "0000001010011001" => 	AC(0) <= "0010000000010011";
						WHEN "0000011001010001" => 	AC(0) <= "0010000011010011";
						WHEN "0000010001001011" => 	AC(0) <= "0010000010010011";
						WHEN "0000001011001010" => 	AC(0) <= "0010000001010011";
						WHEN "0000001000010010" => 	AC(0) <= "0010001000011011";
						WHEN "0000010010000001" => 	AC(0) <= "0010001011011011";
						WHEN "0000000001010010" => 	AC(0) <= "0010001010011011";
						WHEN "0000010010001000" => 	AC(0) <= "0010001001011011";
						WHEN "0000001000001011" => 	AC(0) <= "0010010000100011";
						WHEN "0000001011000001" => 	AC(0) <= "0010010011100011";
						WHEN "0000000001011001" => 	AC(0) <= "0010010010100011";
						WHEN "0000011001001000" => 	AC(0) <= "0010010001100011";
						WHEN "0000001000000100" => 	AC(0) <= "0010011000101011";
						WHEN "0000000100000001" => 	AC(0) <= "0010011011101011";
						WHEN "0000000001100000" => 	AC(0) <= "0010011010101011";
						WHEN "0000100000001000" => 	AC(0) <= "0010011001101011";
						
						WHEN OTHERS => AC(0) <= "0000000000000000";
						
					END CASE;
					STATE <= FETCH;

					
				WHEN OTHERS =>
					STATE <= FETCH;          -- If an invalid state is reached, return to FETCH
					
			END CASE;
			INT_REQ_SYNC <= INT_REQ;  -- register interrupt requests to SCOMP's clock.
		END IF;
	END PROCESS;
	
	-- This process monitors the external interrupt pins, setting
	-- some flags if a rising edge is detected, and clearing flags
	-- once the interrupt is acknowledged.
	PROCESS(RESETN, PCINT, INT_ACK, IIE)
	BEGIN
		IF (RESETN = '0') THEN
			INT_REQ <= "0000";  -- clear all interrupts on reset
		ELSE
			FOR i IN 0 TO 3 LOOP -- for each of the 4 interrupt pins
				IF (INT_ACK(i) = '1') OR (IIE(i) = '0') THEN
					INT_REQ(i) <= '0';   -- if acknowledged or masked, clear interrupt
				ELSIF RISING_EDGE(PCINT(i)) THEN
					INT_REQ(i) <= '1';   -- if rising edge on PCINT, request interrupt
				END IF;
			END LOOP;
		END IF;
	END PROCESS;

END a;
