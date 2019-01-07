/*
 * Copyright {C}, 2017 Neuroptics. All Rights Reserved.
 */

#ifndef PY480_REGSETTINGS_H
#define PY480_REGSETTINGS_H

struct reg_value {
	u16 reg;
	u16 val;
};

struct reg_value py480_enable_clk_management1[] = {
	{2, 0x0000},//Monochrome sensor
	{17, 0x2113},//Configure PLL
	{20, 0x0000},//Configure clock management
	{26, 0x2280}, //Configure PLL lock detector
	{27, 0x3D2D},//Configure PLL lock detector
	{32, 0x2004},//PLL input clock
	{8, 0x0000},//Release PLL soft reset
	{16, 0x0003} //Enable PLL
};

struct reg_value py480_enable_clk_management2[] = {
	{9, 0x0000},//Release clock generator Soft Reset
	{32, 0x7006},//Enable logic clock
	{34, 0x0001} //Enable logic blocks
};

struct reg_value py480_required_register_uploads[]= {
	{2, 0x0000},
	{8, 0x0000},
	{9, 0x0000},
	{10, 0x0000},
	{20, 0x0000},
	{24, 0x0001},
	{26, 0x2280},
	{27, 0x3D2D},
	{32, 0x7007},
	{34, 0x0001},
	{40, 0x0003},
	{41, 0x085F},
	{42, 0x4113},
	{43, 0x0518},
	{48, 0x0001},
	{64, 0x0001},
	{65, 0x382B},
	{66, 0x53C8},
	{67, 0x0665},
	{68, 0x0085},
	{69, 0x0888},
	{70, 0x4800},
	{71, 0x8888},
	{72, 0x0117},
	{106, 0x295E},
	{107, 0x0503},
	{112, 0x0007}, //for CMOS mode
	{128, 0x470A},
	{129, 0x8001},
	{130, 0x0000},//value(bl_line_valid_disable) changed to avoid bl_line before vsync and default is 0x0001 bl_line_valid_enable
	{192, 0x0801},
	{194, 0x03E4}, //reverse x and y enabled for demo kit compatibility
	{197, 0x030A},
	{199, 0x0299},
	{200, 0x0350},
	{201, 0x01F4},
	{207, 0x0014},
	{214, 0x0100},
	{215, 0x111F},
	{216, 0x0000},
	{219, 0x0023},
	{220, 0x3C2B},
	{221, 0x004D},
	{224, 0x3E5E},
	{211, 0x0049},
	{216, 0x0000},
	{219, 0x0023},
	{220, 0x3C2B},
	{221, 0x2B4D},
	{230, 0x0299},
	{231, 0x0350},
	{232, 0x01F4},
	{235, 0x00E1},
	
	//program space
	{384, 0xC800},
	{385, 0xFB1F},
	{386, 0xFB1F},
	{387, 0xFB12},
	{388, 0xF912},
	{389, 0xF903},
	{390, 0xF802},
	{391, 0xF30F},
	{392, 0xF30F},
	{393, 0xF30F},
	{394, 0xF30A},
	{395, 0xF101},
	{396, 0xF00A},
	{397, 0xF24B},
	{398, 0xF201},
	{399, 0xF226},
	{400, 0xF021},
	{401, 0xF001},
	{402, 0xF402},
	{403, 0xF007},
	{404, 0xF20F},
	{405, 0xF20F},
	{406, 0xF202},
	{407, 0xF006},
	{408, 0xEC08},
	{409, 0xC801},
	{410, 0xC800},
		
	{419, 0xC800},
	{420, 0xCC02},
	{421, 0xCC01},
	{422, 0xCC02},
	{423, 0xCC01},
	{424, 0xCC02},
	{425, 0xC805},
	{426, 0xC800},
	
	{427, 0x0030},
	{428, 0x207B},
	{429, 0x2071},
	{430, 0x0071},
	{431, 0x107F},
	{432, 0x1072},
	{433, 0x1074},
	{434, 0x0071},
	{435, 0x0031},
	{436, 0x21BB},
	{437, 0x20B1},
	{438, 0x00B1},
	{439, 0x10BF},
	{440, 0x10B2},
	{441, 0x10B4},
	{442, 0x00B1},
	{443, 0x0030},
	
	{444, 0x0030},
	{445, 0x217B},
	{446, 0x2071},
	{447, 0x0071},
	{448, 0x107F},
	{449, 0x1072},
	{450, 0x1074},
	{451, 0x0071},
	{452, 0x0031},
 	{453, 0x21BB},
	{454, 0x20B1},
	{455, 0x00B1},
	{456, 0x10BF},
	{457, 0x10B2},
	{458, 0x10B4},
	{459, 0x00B1},
	{460, 0x0030},
	
	{461, 0x0030},
	{462, 0x217B},
	{463, 0x2071},
	{464, 0x0071},
	{465, 0x1071},
	{466, 0x0071},
	{467, 0x0031},
	{468, 0x21BB},
	{469, 0x20B1},
	{470, 0x00B1},
	{471, 0x10B3},
	{472, 0x10B1},
	{473, 0x00B1},
	{474, 0x003F},
	{475, 0x0032},
	{476, 0x0030}
};

struct reg_value py480_reso_808x608_60[] = {
	{256, 0xC900},	// ROI0 X config
	{257, 0x9700},	// ROI0 Y config
	{199, 0x0514},	// Mult timer 0
	{201, 0x01F4}	// Exposure 0
};

struct reg_value py480_reso_640x480_60[] = {
	/*changed register value for center ROI*/
	{256, 0xB718},	// ROI0 X config
	{257, 0x850E},	// ROI0 Y config
	{199, 0x0514},	// Mult timer 0
	{201, 0x01F4}	// Exposure 0

};

struct reg_value py480_reso_320x240_60[] = {
	/*changed register value for center ROI*/
	{256, 0x8C3D},	// ROI0 X config
	{257, 0x692E},	// ROI0 Y config
	{199, 0x0514},	// Mult timer 0
	{201, 0x01F4}	// Exposure 0
};

struct reg_value py480_reso_808x608_30[] = {
	{256, 0xC900},	// ROI0 X config
	{257, 0x9700},	// ROI0 Y config
	{199, 0x0a28},	// Mult timer 0
	{201, 0x01f4}	// Exposure 0
};

struct reg_value py480_reso_640x480_30[] = {
	/*changed register value for center ROI*/
	{256, 0xB718},	// ROI0 X config
	{257, 0x850E},	// ROI0 Y config
	{199, 0x0a28},	// Mult timer 0
	{201, 0x01f4}	// Exposure 0
};

struct reg_value py480_reso_320x240_30[] = {
	/*changed register value for center ROI*/
	{256, 0x8C3D},	// ROI0 X config
	{257, 0x692E},	// ROI0 Y config
	{199, 0x0a28},	// Mult timer 0
	{201, 0x01f4}	// Exposure 0
};

struct reg_value py480_gain_10[] = {
	/*total gain 10*/
	{204, 0x01E4},	// AGAIN - default analog gain 2
	{205, 0x0280},	// DGAIN - default digital gain 5
};

struct reg_value py480_soft_power_up[] = {
	{10, 0x0000},	// Release soft reset state
	{32, 0x7007},	// Enable analog clock
	{40, 0x0003},	// Enable column multiplexer
	{42, 0x4113},	// Configure image core
	{48, 0x0001},	// Enable AFE
	{64, 0x0001},	// Enable biasing block
	{72, 0x0117},	// Enable charge pump
	{112, 0x0007},  // Enable LVDS transmitter
};

struct reg_value py480_soft_power_down[] = {
	{72, 0x0999},	// soft reset 
	{64, 0x7006},	// Disable analog clock
	{48, 0x0000},	// Disable column multiplexer
	{42, 0x4110},	// Configure image core
	{40, 0x0000},	// Disable AFE
	{32, 0x0000},	// Disable biasing block
	{10, 0x0010}	// Disable charge pump
};

struct reg_value py480_disable_clock_management2[] = {
	{34, 0x0000}, //Soft reset clock generator
	{32, 0x7004}, //Disable logic clock
	{9,  0x0000}  //Disable logic blocks
};

struct reg_value py480_disable_clock_management1[] = {
	{16, 0x0099}, //Soft reset PLL
	{8,  0x0000}, //Disable PLL
};
#endif
