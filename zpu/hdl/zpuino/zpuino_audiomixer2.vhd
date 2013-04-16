--
-- Audio mixer
--
-- Copyright 2013 Alvaro Lopes
--
-- Version: 0.2
--
-- The FreeBSD license
-- 
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
-- 
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above
--    copyright notice, this list of conditions and the following
--    disclaimer in the documentation and/or other materials
--    provided with the distribution.
-- 
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY
-- EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
-- PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
-- ZPU PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
-- INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
-- (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
-- STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
-- ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--

library ieee;
  use ieee.std_logic_1164.all;
  --use ieee.std_logic_arith.all;
  use ieee.std_logic_unsigned.all;
  --use ieee.std_logic_signed.all;
  use ieee.numeric_std.all;
library work;
  use work.zpuino_config.all;
  use work.zpu_config.all;
  use work.zpupkg.all;

entity zpuino_audiomixer2 is
  generic (
    bits: integer := 18;
    volbits: integer := 16;
    entries: integer range 2 to 8 := 8;
    DISABLE_VOLUME: boolean := FALSE -- For simulations
  );
	port (
    wb_clk_i:   in std_logic;
	 	wb_rst_i:   in std_logic;
    wb_dat_o:   out std_logic_vector(wordSize-1 downto 0);
    wb_dat_i:   in std_logic_vector(wordSize-1 downto 0);
    wb_adr_i:   in std_logic_vector(maxIObit downto minIObit);
    wb_we_i:    in std_logic;
    wb_cyc_i:   in std_logic;
    wb_stb_i:   in std_logic;
    wb_ack_o:   out std_logic;
    wb_inta_o:  out std_logic;

    ena:			  in std_logic;
    data_in1:  	in signed(bits-1 downto 0);
    data_in2:  	in signed(bits-1 downto 0);
    data_in3:  	in signed(bits-1 downto 0);
    data_in4:  	in signed(bits-1 downto 0);
    data_in5:  	in signed(bits-1 downto 0);
    data_in6:  	in signed(bits-1 downto 0);
    data_in7:  	in signed(bits-1 downto 0);
    data_in8:  	in signed(bits-1 downto 0);
    
    audio_out: 	out std_logic_vector(1 downto 0)
    );
end entity zpuino_audiomixer2;

architecture behave of zpuino_audiomixer2 is

function s_shr(si: in signed; amount: in integer) return signed is
  variable r: signed(si'HIGH - amount downto 0);
  variable s: signed(si'RANGE);
begin
  s:= shift_right(si,amount);
  r:=s(si'HIGH-amount downto 0);
  return r;
end function;

function s_extend(si: in signed; size: in integer) return signed is
  variable r: signed(size-1 downto 0);
begin
  r(si'HIGH downto si'LOW):=si;
  sext: for i in size-1 downto si'HIGH+1 loop
    r(i):=si(si'HIGH);
  end loop;
  return r;
end function;

-- DAC
component simple_sigmadelta is
  generic (
    BITS: integer := 18
  );
	port (
    clk:      in std_logic;
    rst:      in std_logic;
    data_in:  in std_logic_vector(BITS-1 downto 0);
    data_out: out std_logic
    );
end component simple_sigmadelta;

subtype sampletype is signed(bits-1 downto 0);
type soundtype is array(0 to 1) of sampletype;

subtype outsampletype is unsigned(bits-1 downto 0);
type outsoundtype is array(0 to 1) of outsampletype;

type sound_in_type is array(0 to entries-1) of sampletype;
signal voice: sound_in_type;

subtype voltype is unsigned(volbits-1 downto 0);
type volumes_type is array(0 to entries-1) of voltype;
type chanvolumes_type is array(0 to 1) of volumes_type;

constant SMAX: integer := (2**(bits-1))-1;
constant SMIN: integer := -1 - SMAX;

type statetype is (
  idle,
  update,
  n1,
  n2,
  clip
);

type regstype is record
  vol: chanvolumes_type;
  ci: integer range 0 to entries-1;
  lr: integer range 0 to 1;
  acc: signed(31 downto 0);
  state: statetype;
  mul: signed((bits*2)-1 downto 0);
  sound: soundtype;
  ack: std_logic;
  sign: std_logic;
  dat_o: std_logic_vector(31 downto 0);
end record;

signal r: regstype;
signal outsound: outsoundtype;
signal soundadapt: unsigned(bits-1 downto 0);

begin

voice(0) <= signed(data_in1);
voice(1) <= signed(data_in2);
v2: if entries>2 generate
voice(2) <= signed(data_in3);
end generate;
v3: if entries>3 generate
voice(3) <= signed(data_in4);
end generate;
v4: if entries>4 generate
voice(4) <= signed(data_in5);
end generate;
v5: if entries>5 generate
voice(5) <= signed(data_in6);
end generate;
v6: if entries>6 generate
voice(6) <= signed(data_in7);
end generate;
v7: if entries>7 generate
voice(7) <= signed(data_in8);
end generate;

wb_inta_o<='0';
wb_ack_o <= r.ack;
wb_dat_o <= r.dat_o;

process(r,wb_clk_i,wb_rst_i, r,voice, wb_cyc_i, wb_stb_i, wb_we_i, wb_adr_i, wb_dat_i)
 variable w: regstype;
 variable exvol: signed(bits-1 downto 0);
 variable ch: integer;
 variable idx: integer;
 variable smv: signed(bits downto 0);
 variable acc_r: signed(31 downto 0);
begin
  w := r;
  w.ack := '0';

  case r.state is
    when idle =>
      w.ci := entries-1;
      w.acc := (others => '0');
      w.state := n1;

    when n1 =>
      exvol := (others => '0');
      exvol(volbits-1 downto 0) := signed(r.vol(r.lr)(r.ci));
      w.mul :=  exvol * voice(r.ci);
      w.state := n2;

    when n2 =>
      -- Allow *4, so we can max signal
      w.acc := r.acc + s_extend(s_shr(r.mul,volbits-2), r.acc'LENGTH);
      if r.ci=0 then
        --r.si <= entries-1;
        if r.lr=0 then
          w.lr:=1;
        else
          w.lr:=0;
        end if;
        w.state := clip;
      else
        w.ci := r.ci - 1;
        w.state := n1;
      end if;

    when clip =>
      -- Divide by 8.
      acc_r := shift_right(r.acc, 3);
      if acc_r > SMAX then
        w.acc := to_signed(SMAX, w.acc'LENGTH);
      elsif acc_r < SMIN then
        w.acc := to_signed(SMIN, w.acc'LENGTH);
      else
        w.acc := acc_r;
      end if;

      w.state := update;
    when update =>

      -- clip if needed

      w.sound(r.lr)(bits-2 downto 0) := r.acc(bits-2 downto 0);
      w.sound(r.lr)(bits-1) := r.acc(r.acc'HIGH);

      w.state := idle;

    when others =>
  end case;

  -- WB access
  if wb_cyc_i='1' and wb_stb_i='1' and r.ack='0' then
    idx := conv_integer(wb_adr_i(4 downto 2));
    if wb_adr_i(5)='1' then
      ch:=1;
    else
      ch:=0;
    end if;
    --ch := conv_integer(wb_adr_i(5));
    if wb_we_i='1' then
      w.vol(ch)(idx) := unsigned(wb_dat_i(voltype'RANGE));
    end if;
    w.dat_o := (others => '0');
    w.dat_o(voltype'RANGE) := std_logic_vector(r.vol(ch)(idx));
    w.ack := '1';
  end if;

  if wb_rst_i='1' then
    w.ci := entries - 2;
    w.lr := 1;
    w.state := idle;
    w.ack := '0';
    w.vol := (others => (others => (others => '0')));
    w.sound := (others => (others => '0'));
  end if;

  if rising_edge(wb_clk_i) then
    r<=w;
  end if;
end process;

  soundadapt(bits-2 downto 0) <= (others => '0');
  soundadapt(bits-1) <= '1';

  outputs: for i in 0 to 1
  generate

  outsound(i) <= unsigned(r.sound(i)) - soundadapt;

	sd: simple_sigmadelta
	generic map (
		BITS =>  bits
	)
	port map (
		clk       => wb_clk_i,
		rst       => wb_rst_i,
		data_in   => std_logic_vector(outsound(i)),
		data_out  => audio_out(i)
	);
  end generate;

end behave;

