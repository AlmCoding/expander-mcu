
extern "C" {

#include "driver/tf/FrameDriver.hpp"
#include "tf/TinyFrame.h"
#include "util/debug.hpp"

#define DEBUG_ENABLE_TF_INTEGRATION 1
#if ((DEBUG_ENABLE_TF_INTEGRATION == 1) && (ENABLE_RTT_DEBUG_OUTPUT == 1))
#define DEBUG_INFO(f, ...) util::dbg::print(util::dbg::TERM0, "[INF][TF_int]: " f "\n", ##__VA_ARGS__);
#define DEBUG_WARN(f, ...) util::dbg::print(util::dbg::TERM0, "[WRN][TF_int]: " f "\n", ##__VA_ARGS__);
#define DEBUG_ERROR(f, ...) util::dbg::print(util::dbg::TERM0, "[ERR][TF_int]: " f "\n", ##__VA_ARGS__);
#else
#define DEBUG_INFO(...)
#define DEBUG_WARN(...)
#define DEBUG_ERROR(...)
#endif

/**
 * This is an example of integrating TinyFrame into the application.
 *
 * TF_WriteImpl() is required, the mutex functions are weak and can
 * be removed if not used. They are called from all TF_Send/Respond functions.
 *
 * Also remember to periodically call TF_Tick() if you wish to use the
 * listener timeout feature.
 */
void TF_WriteImpl(TinyFrame* /*tf*/, const uint8_t* buff, uint32_t len) {
  auto& tf_driver = driver::tf::FrameDriver::getInstance();
  tf_driver.sendData(buff, len);
}

// --------- Mutex callbacks ----------
// Needed only if TF_USE_MUTEX is 1 in the config file.
// DELETE if mutex is not used

/*
// Claim the TX interface before composing and sending a frame
bool TF_ClaimTx(TinyFrame* tf) {
  // take mutex
  return true;  // we succeeded
}

// Free the TX interface after composing and sending a frame
void TF_ReleaseTx(TinyFrame* tf){
  // release mutex
}
*/

// --------- Custom checksums ---------
// This should be defined here only if a custom checksum type is used.
// DELETE those if you use one of the built-in checksum types

/*
// Initialize a checksum
TF_CKSUM TF_CksumStart(void) {
  return 0;
}

// Update a checksum with a byte
TF_CKSUM TF_CksumAdd(TF_CKSUM cksum, uint8_t byte) {
  return cksum ^ byte;
}

// Finalize the checksum calculation
TF_CKSUM TF_CksumEnd(TF_CKSUM cksum) {
  return cksum;
}
*/

}  // extern "C"
