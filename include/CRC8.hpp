#ifndef INCLUDE_CRC8_HPP
#define INCLUDE_CRC8_HPP

#include <cstdint>
#include <cstddef>
class CRC8
{
 private:
  static const uint8_t INIT = 0xFF;  ///< CRC8 初始值 / CRC8 initial value

 public:
  static inline uint8_t tab_[256];  ///< CRC8 查找表 / CRC8 lookup table
  static inline bool inited_ =
      false;  ///< 查找表是否已初始化 / Whether the lookup table is initialized

  CRC8() {}

  /**
   * @brief 生成 CRC8 查找表 / Generates the CRC8 lookup table
   */
  static void GenerateTable()
  {
    uint8_t crc = 0;

    for (int i = 0; i < 256; i++)
    {
      tab_[i] = i;
    }

    for (int i = 0; i < 256; i++)
    {
      for (int j = 7; j >= 0; j--)
      {
        crc = tab_[i] & 0x01;

        if (crc)
        {
          tab_[i] = tab_[i] >> 1;
          tab_[i] ^= 0x8c;
        }
        else
        {
          tab_[i] = tab_[i] >> 1;
        }
      }
    }
    inited_ = true;
  }

  /**
   * @brief 计算数据的 CRC8 校验码 / Computes the CRC8 checksum for the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 计算得到的 CRC8 值 / Computed CRC8 value
   */
  static uint8_t Calculate(const void *raw, size_t len)
  {
    const uint8_t *buf = reinterpret_cast<const uint8_t *>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    uint8_t crc = INIT;

    while (len-- > 0)
    {
      crc = tab_[(crc ^ *buf++) & 0xff];
    }

    return crc;
  }

  /**
   * @brief 验证数据的 CRC8 校验码 / Verifies the CRC8 checksum of the given data
   * @param raw 输入数据指针 / Pointer to input data
   * @param len 数据长度 / Length of the data
   * @return 校验成功返回 `true`，否则返回 `false` /
   *         Returns `true` if the checksum is valid, otherwise returns `false`
   */
  static bool Verify(const void *raw, size_t len)
  {
    const uint8_t *buf = reinterpret_cast<const uint8_t *>(raw);
    if (!inited_)
    {
      GenerateTable();
    }

    if (len < 2)
    {
      return false;
    }
    uint8_t expected = Calculate(buf, len - sizeof(uint8_t));
    return expected == buf[len - sizeof(uint8_t)];
  }
  template<typename Packet>

  static bool Check(const Packet& data)
  {
    const void* ptr = &data;
    return Verify(ptr, sizeof(Packet));
  }
};
#endif