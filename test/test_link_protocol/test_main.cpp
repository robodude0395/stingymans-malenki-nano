// Host unit tests for include/link_protocol.h — run with: pio test -e native
#include <unity.h>
#include "../../include/link_protocol.h"

void setUp() {}
void tearDown() {}

static bool inPool(uint8_t ch) {
  for (uint8_t i = 0; i < CHANNEL_POOL_SIZE; i++) {
    if (CHANNEL_POOL[i] == ch) return true;
  }
  return false;
}

static void checkTable(uint32_t id) {
  LinkParams lp;
  deriveLinkParams(id, lp);
  uint8_t violations = 0;
  for (uint8_t i = 0; i < HOP_COUNT; i++) {
    uint8_t ch = lp.hop[i];
    TEST_ASSERT_TRUE(inPool(ch));
    TEST_ASSERT_NOT_EQUAL(2, ch);
    TEST_ASSERT_NOT_EQUAL(26, ch);
    TEST_ASSERT_NOT_EQUAL(76, ch);
    TEST_ASSERT_NOT_EQUAL(80, ch);
    for (uint8_t j = 0; j < i; j++) TEST_ASSERT_NOT_EQUAL(lp.hop[j], ch);
    uint8_t next = lp.hop[(i + 1) % HOP_COUNT];
    if (lpAbsDiff(ch, next) < MIN_HOP_SPACING) violations++;
  }
  TEST_ASSERT_LESS_OR_EQUAL(1, violations);
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t b = lp.address[i];
    TEST_ASSERT_TRUE(b != 0x00 && b != 0x55 && b != 0xAA && b != 0xFF);
  }
}

void test_deterministic() {
  LinkParams a, b;
  deriveLinkParams(0x5EED1234UL, a);
  deriveLinkParams(0x5EED1234UL, b);
  TEST_ASSERT_EQUAL_MEMORY(a.address, b.address, sizeof(a.address));
  TEST_ASSERT_EQUAL_MEMORY(a.hop, b.hop, sizeof(a.hop));
  TEST_ASSERT_EQUAL_UINT16(a.idTag, b.idTag);
  TEST_ASSERT_EQUAL_UINT8(a.key, b.key);
}

void test_hop_tables() {
  for (uint32_t id = 0; id < 1000; id++) checkTable(id);
  const uint32_t big[] = {0x5EED1234UL, 0xFFFFFFFFUL, 0x80000000UL, 0xA5C35A3CUL, 0x9E3779B9UL, 0x1F2E3D4CUL};
  for (uint32_t id : big) checkTable(id);
}

void test_unique_addresses() {
  static uint8_t addrs[1000][5];
  for (uint32_t id = 0; id < 1000; id++) {
    LinkParams lp;
    deriveLinkParams(id, lp);
    memcpy(addrs[id], lp.address, 5);
    for (uint32_t k = 0; k < id; k++) {
      TEST_ASSERT_FALSE(memcmp(addrs[k], addrs[id], 5) == 0);
    }
  }
}

void test_crc8_check_value() {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX8(0xF4, crc8(s, 9, 0x00));
}

static CommandPacket makePacket(const LinkParams& lp) {
  CommandPacket p;
  p.seq = 42;
  p.hopIdx = 3;
  p.speed = -57;
  p.turn = 88;
  p.flags = CMD_FLAG_BUTTON;
  sealCommandPacket(p, lp);
  return p;
}

void test_packet_validation() {
  LinkParams lp, other;
  deriveLinkParams(0x5EED1234UL, lp);
  deriveLinkParams(0x12345678UL, other);

  CommandPacket p = makePacket(lp);
  TEST_ASSERT_EQUAL(PKT_OK, validateCommandPacket(p, lp));

  for (uint8_t i = 0; i < sizeof(CommandPacket); i++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      CommandPacket q = p;
      ((uint8_t*)&q)[i] ^= (uint8_t)(1u << bit);
      PacketStatus st = validateCommandPacket(q, lp);
      TEST_ASSERT_TRUE(st == PKT_BAD_ID || st == PKT_BAD_CRC);
    }
  }

  CommandPacket foreign = makePacket(other);
  TEST_ASSERT_NOT_EQUAL(PKT_OK, validateCommandPacket(foreign, lp));

  CommandPacket bad = p;
  bad.hopIdx = HOP_COUNT;
  sealCommandPacket(bad, lp);
  TEST_ASSERT_EQUAL(PKT_BAD_FIELD, validateCommandPacket(bad, lp));

  bad = p;
  bad.speed = 101;
  sealCommandPacket(bad, lp);
  TEST_ASSERT_EQUAL(PKT_BAD_FIELD, validateCommandPacket(bad, lp));
}

void test_seq_is_newer() {
  TEST_ASSERT_TRUE(seqIsNewer(1, 0));
  TEST_ASSERT_FALSE(seqIsNewer(0, 0));
  TEST_ASSERT_TRUE(seqIsNewer(0, 255));
  TEST_ASSERT_FALSE(seqIsNewer(100, 200));
}

void test_seq_accept() {
  uint8_t last = 0, restart = 0;
  bool pending = false;

  // Not locked (ACQUIRE): any seq is accepted and becomes the anchor
  TEST_ASSERT_TRUE(seqAccept(200, false, last, restart, pending));
  TEST_ASSERT_EQUAL_UINT8(200, last);

  // Locked: newer accepted, duplicate and old rejected
  TEST_ASSERT_TRUE(seqAccept(201, true, last, restart, pending));
  TEST_ASSERT_FALSE(seqAccept(201, true, last, restart, pending));
  TEST_ASSERT_FALSE(pending);                 // duplicate never arms restart
  TEST_ASSERT_TRUE(seqAccept(0, true, last, restart, pending));   // wrap 201 -> 0 is newer
  last = 201;

  // Single stale packet: rejected, then normal traffic clears the restart state
  TEST_ASSERT_FALSE(seqAccept(150, true, last, restart, pending));
  TEST_ASSERT_TRUE(pending);
  TEST_ASSERT_TRUE(seqAccept(202, true, last, restart, pending));
  TEST_ASSERT_FALSE(pending);
  TEST_ASSERT_EQUAL_UINT8(202, last);

  // TX reboot (seq restarts near 0 while lastSeq = 202, i.e. "behind"):
  // first packet rejected, a confirming newer one re-anchors
  TEST_ASSERT_FALSE(seqAccept(130, true, last, restart, pending));
  TEST_ASSERT_FALSE(seqAccept(130, true, last, restart, pending));  // duplicate of restart seq
  TEST_ASSERT_TRUE(pending);
  TEST_ASSERT_EQUAL_UINT8(202, last);
  TEST_ASSERT_TRUE(seqAccept(132, true, last, restart, pending));   // one packet lost: still ok
  TEST_ASSERT_FALSE(pending);
  TEST_ASSERT_EQUAL_UINT8(132, last);
  TEST_ASSERT_TRUE(seqAccept(133, true, last, restart, pending));

  // Two unrelated old packets going backwards never re-anchor
  last = 100; pending = false;
  TEST_ASSERT_FALSE(seqAccept(20, true, last, restart, pending));
  TEST_ASSERT_FALSE(seqAccept(10, true, last, restart, pending));
  TEST_ASSERT_EQUAL_UINT8(100, last);
}

void test_channel_to_pulse() {
  TEST_ASSERT_EQUAL_UINT16(1000, channelToPulseUs(-100));
  TEST_ASSERT_EQUAL_UINT16(1500, channelToPulseUs(0));
  TEST_ASSERT_EQUAL_UINT16(2000, channelToPulseUs(100));
  TEST_ASSERT_EQUAL_UINT16(1000, channelToPulseUs(-128));
  TEST_ASSERT_EQUAL_UINT16(2000, channelToPulseUs(127));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_deterministic);
  RUN_TEST(test_hop_tables);
  RUN_TEST(test_unique_addresses);
  RUN_TEST(test_crc8_check_value);
  RUN_TEST(test_packet_validation);
  RUN_TEST(test_seq_is_newer);
  RUN_TEST(test_seq_accept);
  RUN_TEST(test_channel_to_pulse);
  return UNITY_END();
}
