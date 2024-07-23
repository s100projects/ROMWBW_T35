-- This file is copyright by Grant Searle 2014
-- You are free to use this file in your own projects but must never charge for it nor use it without
-- acknowledgement.
-- Please ask permission from Grant Searle before republishing elsewhere.
-- If you use this file or any part of it, please add an acknowledgement to myself and
-- a link back to my main web site http://searle.hostei.com/grant/    
-- and to the "multicomp" page at http://searle.hostei.com/grant/Multicomp/index.html
--
-- Please check on the above web pages to see if there are any updates before using this file.
-- If for some reason the page is no longer available, please search for "Grant Searle"
-- on the internet to see if I have moved to another web hosting service.
--
-- Grant Searle
-- eMail address available on my main web page link above.

library ieee;
use ieee.std_logic_1164.all;
use  IEEE.STD_LOGIC_ARITH.all;
use  IEEE.STD_LOGIC_UNSIGNED.all;

entity Microcomputer is
	port(
		n_reset		: in std_logic;
		clk			: in std_logic;
		
		n_wr			: out std_logic;		
		n_rd			: out std_logic;

		n_mreq		: out std_logic;
		n_iorq		: out std_logic;

		
		address		: out std_logic_vector(15 downto 0);
		dataOut		: out	std_logic_vector(7 downto 0);
		dataIn		: in  std_logic_vector(7 downto 0);		
	
		n_wait		: in  std_logic;
		n_int			: in  std_logic;
		n_nmi			: in  std_logic;
	   n_busrq		: in  std_logic;
		n_busak		: out std_logic;

		n_halt		: out std_logic;
		n_rfsh		: out std_logic;
		n_m1			: out std_logic;
        clkcpu      : out std_logic

		);
end Microcomputer;

architecture struct of Microcomputer is

		signal cpuClkCount		: std_logic_vector(5 downto 0);
		signal cpuClock			: std_logic;
	

begin

clkcpu <=  cpuClock;	
	
-- ____________________________________________________________________________________
-- CPU CHOICE GOES HERE
cpu1 : entity work.t80s
generic map(mode => 0, t2write => 1, iowait => 1)
port map(
	reset_n 	=> n_reset,
	clk_n 	=> cpuClock,
	wait_n 	=> n_wait,
	int_n 	=> n_int,
	nmi_n 	=> n_nmi,
	busrq_n 	=> n_busrq,
	m1_n		=> n_m1,
	mreq_n 	=> n_mreq,
	iorq_n 	=> n_iorq,
	rd_n 		=> n_rd,
	wr_n		=> n_wr,
	rfsh_n	=> n_rfsh,
	halt_n	=> n_halt,
	busak_n	=> n_busak,
	a	 		=> Address,
	di 		=> dataIn,
	do 		=> dataOut);
-- ____________________________________________________________________________________
-- ____________________________________________________________________________________
-- SYSTEM CLOCKS GO HERE
-- SUB-CIRCUIT CLOCK SIGNALS 

process (clk)
begin
if rising_edge(clk) then

if cpuClkCount < 4 then -- 4 = 10MHz, 3 = 12.5MHz, 2=16.6MHz, 1=25MHz
cpuClkCount <= cpuClkCount + 1;
else
cpuClkCount <= (others=>'0');
end if;
if cpuClkCount < 2 then -- 2 when 10MHz, 2 when 12.5MHz, 2 when 16.6MHz, 1 when 25MHz
cpuClock <= '0';
else
cpuClock <= '1';
end if;

end if;
end process;




end;
