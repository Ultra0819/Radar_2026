# 来源于 0x020E / 0x0105 / 0x0209
uint8 vulnerability_chance        # 易伤机会 (0~2)
uint8 enemy_vulnerable_status     # 对方是否正在易伤 (0:未被触发, 1:正在被触发)
uint16 dart_info                  # 0x0105: 最近一次己方飞镖击中的目标 (0~5)
uint32 hight_road                 # 0x0209: 对方中央高地跨越增益点 RFID 掩码，bit0 下方，bit1 上方
uint32 enemy_buff                 # 0x0209: 对方堡垒增益点 RFID 是否检测到 (0/1)
