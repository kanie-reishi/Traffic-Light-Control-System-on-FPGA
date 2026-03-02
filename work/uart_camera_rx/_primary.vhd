library verilog;
use verilog.vl_types.all;
entity uart_camera_rx is
    generic(
        CLK_FREQ        : integer := 50000000;
        BAUD_RATE       : integer := 115200;
        TIMEOUT_MS      : integer := 500
    );
    port(
        clk             : in     vl_logic;
        rst_n           : in     vl_logic;
        uart_rx         : in     vl_logic;
        ns_density_o    : out    vl_logic_vector(3 downto 0);
        ew_density_o    : out    vl_logic_vector(3 downto 0);
        camera_valid_o  : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of CLK_FREQ : constant is 1;
    attribute mti_svvh_generic_type of BAUD_RATE : constant is 1;
    attribute mti_svvh_generic_type of TIMEOUT_MS : constant is 1;
end uart_camera_rx;
