use_utool 1
use_uframe 0

TP_GROUPMASK = "1,1,*,*,*"

default.group(1).pose -> [0,0,0,90,180,0]
default.group(1).config -> ['F','U','T', 0, 0, 0]
default.group(2).joints -> [0]

perch_pos := P[1]
perch_pos.group(1).joints -> [138, -34, 16, -81, 120, 107]
perch_pos.group(2).joints -> [0]

joint_move.to(perch_pos).at(15, '%').term(-1)
