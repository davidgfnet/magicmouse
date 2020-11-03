
#define MAX_RULES              8
#define HDR_MAGIC     0x1337f00d

typedef struct {
	uint32_t magic;             // Some magic header value
	uint32_t version;           // Wipes settings if version mismatch!
	uint32_t cfg_rev;           // Config versioning. Grows from zero
	uint32_t checksum;

	// Impersonated device information
	uint32_t usb_vidpid;        // VID:PID info
	char manufacturer[64];      // Padded with nulls
	char product[64];

	// Enable/Disable signals
	uint8_t toggle_key;         // 0xFF means disabled, 0-2 selects key (numlock...)
	uint8_t toggle_mode;        // See above
	uint8_t def_enabled;        // Enabled on reset?

	// Move rules (max 8)
	uint8_t rule_count;         // Enabled rule count
	struct {
		uint32_t period;        // In milliseconds
		uint8_t rtype;          // 0 mouse, 1 keyboard
		uint8_t rdata;          // For keyboards it's the key, mouse the mov amount
		uint8_t rmode;          // For keyboard: modifier key, mouse mov type
		uint8_t rflags;         // Misc: light led when active ...
	} rules[MAX_RULES];
} t_devconf;

