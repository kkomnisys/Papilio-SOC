--
-- Audio mixer
--
-- Copyright 2013 Alvaro Lopes
--
-- Version: 0.1
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
  use ieee.std_logic_arith.all;
  use ieee.std_logic_unsigned.all;

library work;
  use work.zpuino_config.all;
  use work.zpu_config.all;
  use work.zpupkg.all;

entity zpuino_audiomixer2 is
  generic (
    bits: integer := 17;
    volbits: integer := 8;
    entries: integer range 2 to 8 := 8
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
    data_in1:  	in std_logic_vector(bits-1 downto 0);
    data_in2:  	in std_logic_vector(bits-1 downto 0);
    data_in3:  	in std_logic_vector(bits-1 downto 0);
    data_in4:  	in std_logic_vector(bits-1 downto 0);
    data_in5:  	in std_logic_vector(bits-1 downto 0);
    data_in6:  	in std_logic_vector(bits-1 downto 0);
    data_in7:  	in std_logic_vector(bits-1 downto 0);
    data_in8:  	in std_logic_vector(bits-1 downto 0);
    
    audio_out: 	out std_logic_vector(1 downto 0)
    );
end entity zpuino_audiomixer2;

architecture behave of zpuino_audiomixer2 is

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

subtype sampletype is unsigned(bits-1 downto 0);
type soundtype is array(0 to 1) of sampletype;
type sound_in_type is array(0 to entries-1) of sampletype;
signal voice: sound_in_type;

subtype voltype is unsigned(volbits-1 downto 0);
type volumes_type is array(0 to entries-1) of voltype;
type chanvolumes_type is array(0 to 1) of volumes_type;

type statetype is (
  idle,
  setup,
  pvol,
  addmul,
  comp,
  update
);

type regstype is record
  vol: chanvolumes_type;
  ci: integer range 0 to entries-1;
  lr: integer range 0 to 1;
  acc: unsigned(bits downto 0);
  state: statetype;
  mul: unsigned(((bits+1)*2)-1 downto 0);
  sound: soundtype;
  ack: std_logic;
  dat_o: std_logic_vector(31 downto 0);
end record;

signal r: regstype;

begin

voice(0) <= unsigned(data_in1);
voice(1) <= unsigned(data_in2);
voice(2) <= unsigned(data_in3);
voice(3) <= unsigned(data_in4);
voice(4) <= unsigned(data_in5);
voice(5) <= unsigned(data_in6);
voice(6) <= unsigned(data_in7);
voice(7) <= unsigned(data_in8);

wb_inta_o<='0';
wb_ack_o <= r.ack;
wb_dat_o <= r.dat_o;

process(r,wb_clk_i,wb_rst_i)
 variable w: regstype;
 variable exvol: unsigned(bits downto 0);
 variable ch: integer;
 variable idx: integer;
begin
  w := r;

  case r.state is
    when idle =>
      exvol := (others => '0');
      exvol(volbits-1 downto 0) := r.vol(r.lr)(entries-1);
      w.mul :=  exvol * ( "0" & voice(entries-1));
      w.state := setup;
    when setup =>
      w.acc := r.mul(volbits+bits downto volbits);
      w.state := pvol;
    when pvol =>
      exvol := (others => '0');
      exvol(volbits-1 downto 0) := r.vol(r.lr)(r.ci);
      w.mul :=  exvol * ( "0" & voice(r.ci));
      w.state := addmul;
    when addmul =>
      w.acc := r.acc + r.mul(volbits+bits downto volbits);
      w.mul := r.acc * r.mul(volbits+bits downto volbits);
      w.state := comp;
    when comp =>
      w.acc := r.acc - w.mul((bits*2)-1 downto bits);
      if r.ci=0 then
        w.state := update;
        w.ci := entries - 2;
      else
        w.ci := r.ci - 1;
        w.state := pvol;
      end if;
    when update =>
      if r.acc(bits)='1' then
        w.sound(r.lr) := (others => '1');
      else
        w.sound(r.lr) := r.acc(bits-1 downto 0);
      end if;
      w.state := idle;
      -- Do this in other way, to to support strobe
      if r.lr=0 then w.lr:=1; else w.lr:=0;end if;
    when others =>
  end case;

  -- WB access
  if wb_cyc_i='1' and wb_stb_i='1' and r.ack='0' then
    idx := conv_integer(wb_adr_i(6 downto 2));
    ch := conv_integer(wb_adr_i(7));
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
  end if;

  if rising_edge(wb_clk_i) then
    r<=w;
  end if;
end process;

  outputs: for i in 0 to 1
  generate

	sd: simple_sigmadelta
	generic map (
		BITS =>  bits
	)
	port map (
		clk       => wb_clk_i,
		rst       => wb_rst_i,
		data_in   => std_logic_vector(r.sound(i)),
		data_out  => audio_out(i)
	);
  end generate;

end behave;

