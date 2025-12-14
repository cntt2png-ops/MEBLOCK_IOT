// ================== CORE INCLUDES ==================
#include <MeblockCore.h>
{{INCLUDES}}

// ================== GLOBALS (BLOCKLY) ==============
{{GLOBALS}}

// ================== USER_SETUP (BLOCKLY) ===========
void USER_SETUP() {
{{INIT}}
}

// ================== USER_LOOP (BLOCKLY) ============
void USER_LOOP() {
{{LOOP}}
}

// ================== Arduino setup/loop =============
void setup() {
  meblock_core_setup(115200);
  USER_SETUP();
}

void loop() {
  meblock_core_loop();
  USER_LOOP();
}
