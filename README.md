# IOT DC MOTOR 
int motorPin = 9;
void setup() {
 pinMode(motorPin, OUTPUT);
 Serial.begin(9600);
 while (! Serial);
 Serial.println("Speed 0 to 255");
}
void loop() {
 if (Serial.available()) {
 int speed = Serial.parseInt();
 if (speed >= 0 && speed <= 255) {
 analogWrite(motorPin, speed);
 }
 }
}
Temperature Sensor
Interfacing of Temperature Sensor (LM35):
#include<LiquidCrystal.h>
LiquidCrystal lcd(7,6,5,4,3,2);
void setup()
{
Serial.begin(19200);
lcd.begin(16,2);
}
void loop()
{
int analogvalue = analogRead(A3);
int pecent = map(analogvalue,0,1023,0,600);
Serial.print("analogvalue= ");
Serial.println(analogvalue);
lcd.setCursor(0,0);
lcd.print(analogvalue);
Serial.print("Pecent= ");
Serial.println(pecent);
lcd.setCursor(0,1);
lcd.print(pecent);
delay(1000);
lcd.clear ();
}

Interfacing of IR Sensor:
int ledPin=13;
int inputPin=2;
int val=0;
void setup()
{
pinMode(13,OUTPUT);
pinMode( inputPin, INPUT);
Serial.begin(9600);
}
void loop()
{
val=digitalRead(inputPin); // check the pin status (High=1/Low=0) //Active Low
output
if(val==HIGH)
{
Serial.print(“Object Absent\n");
digitalWrite(13,LOW);
}
else
{
Serial.print("Object Present\n");
digitalWrite(13,HIGH);
}
}

Interfacing of Ultrasonic Sensor:
#include <LiquidCrystal.h>
LiquidCrystal lcd (12,11,5,4,3,2);
// defining the pins
const int trigPin = 10;
const int echoPin = 9;
// defining variables
long duration;
int distance;
void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600); // Starts the serial communication
lcd.begin(16,2);
}
void loop() {
// Clears the trigPin
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
lcd.setCursor (0,0);
lcd.print("Distance: ");
delay(1000);
lcd.setCursor (0,1);
lcd.print(distance);
lcd.print("cm");
}

DHT SENSOR PRACTICAL 5
import sys
import Adafruit_DHT as dht
from time import sleep
import urllib
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.IN) #Smoke Sensor Digital output
GPIO.setwarnings(False)
def DHT11_data():
# Reading from DHT11 and storing the temperature and humidity
humi, temp = dht.read_retry(11, 23)
return humi, temp
# Enter Your API key here
myAPI = "ZQN4Z08XZ9HLOZY0"
# URL where we will send the data, Don't change it
baseURL = "https://api.thingspeak.com/update?api_key=%s" % myAPI
while True:
humidity, temperature = DHT11_data()
print ('Temp: {0:0.1f} C Humidity: {1:0.1f} %'.
format(temperature,humidity))
input_state = GPIO.input(24)
if input_state == False:
smoke = 1
print('Smoke Detected')
print(smoke)
else:
smoke = 0
print('Smoke is not Detected')
print(smoke)
from urllib.request import urlopen
content = urlopen(baseURL + "&field1=%s&field2=%s&field3=%s" %
(temperature,humidity,smoke))
sleep(15);


VLSI EXP 3 MAIN CODE- SHIFT EXP 3 – SHIFT RESISTER
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;
-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;
entity shiftreg is
 Port ( si : in STD_LOGIC;
 clk : in STD_LOGIC;
 so : out STD_LOGIC;
 pin : in STD_LOGIC_VECTOR (3 downto 0);
 po : out STD_LOGIC_VECTOR (3 downto 0);
 sel : in STD_LOGIC_VECTOR (1 downto 0));
end shiftreg;
architecture Behavioral of shiftreg is
signal temp:STD_LOGIC_VECTOR( 3 downto 0);
begin
process(clk)
begin
if(clk'event and clk='1')then
case sel is
when"00"=> temp<= si&temp(3 downto 1);
so <=temp(3);
when"01"=> temp<= si&temp(3 downto 1);
po<=temp;
when others=>null;
end case;
end if;
end process;
end Behavioral;

TESTBENCH
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
ENTITY shifttest IS
END shifttest;
ARCHITECTURE behavior OF shifttest IS
 -- Component Declaration for the Unit Under Test (UUT)
 COMPONENT shiftreg
 PORT(
 si : IN std_logic;
 clk : IN std_logic;
 so : OUT std_logic;
 pin : IN std_logic_vector(3 downto 0);
 po : OUT std_logic_vector(3 downto 0);
 sel : IN std_logic_vector(1 downto 0)
 );
 END COMPONENT;
 --Inputs
 signal si : std_logic := '0';
 signal clk : std_logic := '0';
 signal pin : std_logic_vector(3 downto 0) := (others => '0');
 signal sel : std_logic_vector(1 downto 0) := (others => '0');
--Outputs
 signal so : std_logic;
 signal po : std_logic_vector(3 downto 0);
 -- Clock period definitions
 constant clk_period : time := 10 ns;
BEGIN
-- Instantiate the Unit Under Test (UUT)
 uut: shiftreg PORT MAP (
 si => si,
 clk => clk,
 so => so,
 pin => pin,
 po => po,
 sel => sel
 );
 -- Clock process definitions
 clk_process :process
 begin
 clk <= '0';
wait for clk_period/2;
clk <= '1';
wait for clk_period/2;
 end process;
 -- Stimulus process
 stim_proc: process
 begin
 sel<="00";
 si<='1';
 -- hold reset state for 100 ns.
 wait for 100 ns;
sel<="01";
si<='1';
wait for 100 ns;
sel<="10";
pin<="1100";
wait for 100 ns;
sel<="11";
pin<="1100";
 --=wait for clk_period*10;
 -- insert stimulus here
 wait;
 end process;
END;

VLSI EXP 4 MAIN CODE-MOD –N COUNTER
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_unsigned.ALL;
use IEEE.NUMERIC_STD.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;
-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;
entity mod5ex is
 Port ( clk : in STD_LOGIC;
 clr : in STD_LOGIC;
 q : inout STD_LOGIC_VECTOR (2 downto 0));
end mod5ex;
architecture Behavioral of mod5ex is
signal count: std_logic_vector(2 downto 0);
begin
process(clk)
begin
if (clr='1') then count <= "000";
elsif (rising_edge (clk)) then
if (count="100")
then count <= "000";
else
count<=count+ 1;
end if;
end if;
end process;
q<=count;
end Behavioral;

TESTBENCH
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
ENTITY mod5test IS
END mod5test;
ARCHITECTURE behavior OF mod5test IS
 -- Component Declaration for the Unit Under Test (UUT)
 COMPONENT mod5ex
 PORT(
 clk : IN std_logic;
 clr : IN std_logic;
 q : INOUT std_logic_vector(2 downto 0)
 );
 END COMPONENT;
 --Inputs
 signal clk : std_logic := '0';
 signal clr : std_logic := '0';
--BiDirs
 signal q : std_logic_vector(2 downto 0);
 -- Clock period definitions
 constant clk_period : time := 10 ns;
BEGIN
-- Instantiate the Unit Under Test (UUT)
 uut: mod5ex PORT MAP (
 clk => clk,
 clr => clr,
 q => q
 );
 -- Clock process definitions
 clk_process :process
 begin
clk <= '0';
wait for 10 ns;
clk <= '1';
wait for 10 ns;
 end process;
 -- Stimulus process
 stim_proc: process
 begin
clr<='1';
 -- hold reset state for 100 ns.
 wait for 20 ns;
clr<='0';
 -- hold reset state for 100 ns.
 wait for 20 ns;
 -- hold reset state for 100 ns.
-- wait for 100 ns;
-- wait for clk_period*10
 -- insert stimulus here
 wait;
 end process;

 EXP 5 – FIFO
 library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.numeric_std.all;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;
-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;
entity fifo_vhdl is
 Port ( clk,rst : in STD_LOGIC;
 enr : in STD_LOGIC; --enable read,should be '0' when not in use.
 enw : in STD_LOGIC; --enable write,should be '0' when not in use.
 dataout : out STD_LOGIC_VECTOR (3 downto 0); --output data
 datain : in STD_LOGIC_VECTOR (3 downto 0); --input data
 FF_empty,clk_div : out STD_LOGIC; --set as '1' when the queue is empty
 FF_full : out STD_LOGIC); ---set as '1' when the queue is full
 end fifo_vhdl;
architecture Behavioral of fifo_vhdl is
type memory_type is array (0 to 7) of std_logic_vector(3 downto 0);
signal memory: memory_type :=(others=>(others=> '0'));
signal readptr,writeptr: std_logic_vector(2 downto 0) :="000";
signal count : std_logic_vector(2 downto 0):="000";
signal newclk : std_logic;
signal count1 : std_logic_vector(25 downto 0);
begin
-----------------------------------------------------------------
clk_div1: process(clk,rst)
begin
if rst='1' then
count1<=(others =>'0');
elsif clk'event and clk='1' then
count1<=count1+'1';
end if;
end process;
-----------------------------------------------------------------
newclk<=count1(25);
clk_div<=count1(25);
-----------------------------------------------------------------
fifo_emty_full: process(readptr,writeptr,count)
begin
if(count="000") then
FF_empty<='1';
FF_full<='0';
elsif(count="111")then
FF_empty<='0';
FF_full<='1';
end if;
end process;
count1_reptr_wdptr: process(newclk,rst,enr,enw,readptr,writeptr)
begin
if rst='1' then
count<="000";
readptr<=(others=>'0');
writeptr<=(others=>'0');
else if newclk'event and newclk='1' then
if enw='1' and enr='0' then
count<=count+'1';
if count="111" then
count<=count;
end if;
readptr<=readptr;
writeptr<=writeptr+1;
elsif enw='0' and enr='1' then
count<=count-'1';
if count="000" then
count<=count;
end if;
readptr<=readptr+1;
writeptr<=writeptr;
else
null;
end if;
end if;
end if;
end process;
mem_read_write:process(newclk,count,enw,enr)
begin
if(newclk'event and newclk='1') then
if enw='1' and enr='0' then
if count /="111" then
memory(conv_integer(writeptr))<=datain;
end if;
elsif enw='0' and enr='1' then
if count /="000" then
dataout<=memory(conv_integer(readptr));
end if;
end if;
end if;
end process;
end Behavioral;

TEST BENCH
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
ENTITY fifo_test IS
END fifo_test;
ARCHITECTURE behavior OF fifo_test IS
 -- Component Declaration for the Unit Under Test (UUT)
 COMPONENT fifo_vhdl
 PORT(
 clk : IN std_logic;
 rst : IN std_logic;
 enr : IN std_logic;
 enw : IN std_logic;
 dataout : OUT std_logic_vector(3 downto 0);
 datain : IN std_logic_vector(3 downto 0);
 FF_empty : OUT std_logic;
 clk_div : OUT std_logic;
 FF_full : OUT std_logic
 );
 END COMPONENT;

 --Inputs
 signal clk : std_logic := '0';
 signal rst : std_logic := '0';
 signal enr : std_logic := '0';
 signal enw : std_logic := '0';
 signal datain : std_logic_vector(3 downto 0) := (others => '0');
--Outputs
 signal dataout : std_logic_vector(3 downto 0);
 signal FF_empty : std_logic;
 signal clk_div : std_logic;
 signal FF_full : std_logic;
 -- Clock period definitions
 constant clk_period : time := 10 ns;
 constant clk_div_period : time := 10 ns;
BEGIN
-- Instantiate the Unit Under Test (UUT)
 uut: fifo_vhdl PORT MAP (
 clk => clk,
 rst => rst,
 enr => enr,
 enw => enw,
 dataout => dataout,
 datain => datain,
 FF_empty => FF_empty,
 clk_div => clk_div,
 FF_full => FF_full
 );
 -- Clock process definitions
 clk_process :process
 begin
clk <= '0';
wait for clk_period/2;
clk <= '1';
wait for clk_period/2;
 end process;
 clk_div_process :process
 begin
clk_div <= '0';
wait for clk_div_period/2;
clk_div <= '1';
wait for clk_div_period/2;
 end process;
-- Stimulus process
 stim_proc: process
 begin
 -- hold reset state for 100 ns.
rst<='1';
enr<='0';
enw<='1';
dataout<="0110";
wait for 100 ns;
rst<='0';
enr<='0';
enw<='1';
dataout<="1100";
wait for 100 ns;
rst<='0';
enr<='1';
enw<='0';
dataout<="1100";
wait for 100 ns;
 wait for clk_period*10;
 -- insert stimulus here
 wait;
 end process;
END;

EXP 1
Half Adder
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity adder is
    Port ( a : in  STD_LOGIC;
           b : in  STD_LOGIC;
           s : out  STD_LOGIC;
           c : out  STD_LOGIC);
end adder;

architecture Behavioral of adder is

begin
s<=a xor b;
c<=a and b;

end Behavioral;


TESTBENCH
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY hadder IS
END hadder;
 
ARCHITECTURE behavior OF hadder IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT adder
    PORT(
         a : IN  std_logic;
         b : IN  std_logic;
         s : OUT  std_logic;
         c : OUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal a : std_logic := '0';
   signal b : std_logic := '0';

 	--Outputs
   signal s : std_logic;
   signal c : std_logic;
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
   --constant <clock>_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: adder PORT MAP (
          a => a,
          b => b,
          s => s,
          c => c
        );

   -- Clock process definitions
--   <clock>_process :process
--   begin
--		<clock> <= '0';
--		wait for <clock>_period/2;
--		<clock> <= '1';
--		wait for <clock>_period/2;
--   end process;
-- 

   -- Stimulus process
   stim_proc: process
   begin	
a<='0';
b<='0';	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
a<='0';
b<='1';	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
a<='1';
b<='0';	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
a<='1';
b<='1';	
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      --wait for <clock>_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;

EXP-2
VHDL CODE:
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_unsigned.ALL;
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity bitalu is
    Port ( a : in  STD_LOGIC_VECTOR (3 downto 0);
           b : in  STD_LOGIC_VECTOR (3 downto 0);
           sel : in  STD_LOGIC_VECTOR (2 downto 0);
           d : out  STD_LOGIC_VECTOR (3 downto 0));
end bitalu;

architecture Behavioral of bitalu is

begin
process(a,b,sel)
begin
case sel is
when "000"=> d<= a+b;
when "001"=> d<= a-b;
when "010"=> d<= a and b;
when "011"=> d<= a nand b;
when "100"=> d<= a or b;
when "101"=> d<= a nor b;
when "110"=> d<= a xor b;
when "111"=> d<= a;
when others=> null;
end case;
end process;
end Behavioral;

TESTBENCH
TEST BENCH:
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY ddd IS
END ddd;
 
ARCHITECTURE behavior OF ddd IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT bitalu
    PORT(
         a : IN  std_logic_vector(3 downto 0);
         b : IN  std_logic_vector(3 downto 0);
         sel : IN  std_logic_vector(2 downto 0);
         d : OUT  std_logic_vector(3 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal a : std_logic_vector(3 downto 0) := (others => '0');
   signal b : std_logic_vector(3 downto 0) := (others => '0');
   signal sel : std_logic_vector(2 downto 0) := (others => '0');

 	--Outputs
   signal d : std_logic_vector(3 downto 0);
   -- No clocks detected in port list. Replace <clock> below with 
   -- appropriate port name 
 
--   constant <clock>_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: bitalu PORT MAP (
          a => a,
          b => b,
          sel => sel,
          d => d
        );

   -- Clock process definitions
--   <clock>_process :process
--   begin
--		<clock> <= '0';
--		wait for <clock>_period/2;
--		<clock> <= '1';
--		wait for <clock>_period/2;
--   end process;
-- 

   -- Stimulus process
   stim_proc: process
   begin	
a<="1100";
b<="1010";
sel<="000";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
a<="1100";
b<="1010";
sel<="001";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		a<="1100";
b<="1010";
sel<="010";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		a<="1100";
b<="1010";
sel<="011";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		a<="1100";
b<="1010";
sel<="100";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		a<="1100";
b<="1010";
sel<="101";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		a<="1100";
b<="1010";
sel<="110";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
		a<="1100";
b<="1010";
sel<="111";	
      -- hold reset state for 100 ns.
      wait for 100 ns;	
--      wait for <clock>_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;







