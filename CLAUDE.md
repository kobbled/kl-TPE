# TPE — CLAUDE.md (AI Context)

## Purpose

TPE is a Layer 5 module providing the bridge between Karel programs and FANUC TP (Teach Pendant) programs. It has two responsibilities:
1. **tpefile** — low-level TP program lifecycle (open/close/create/copy/write) and AR[] parameter extraction from TP callers.
2. **tpepose** — OOP class for reading and writing P[] position registers inside open TP programs, parsing LS motion statement lines, and composing/copying pose paths.

Every Ka-Boost module that exposes a TP interface (`tp/*.kl` programs) uses `tpe__get_int_arg` / `tpe__get_string_arg` etc. to read their AR[] arguments. The `tpepose` class is used by the `paths` pipeline (pathmake, pathmotion) to programmatically modify TP programs at runtime.

---

## Repository Layout

```
lib/TPE/
├── package.json                       rossum manifest (project: TPElib)
├── readme.md                          developer reference (generated)
├── CLAUDE.md                          this file
├── lib/
│   ├── tpefile/
│   │   ├── tpelib.kl                  all tpe__ routine implementations (442 lines)
│   │   ├── tpe.klh                    public header — declare_function entries
│   │   ├── tpe.private.klh            private header (placeholder — empty)
│   │   ├── tpe.klt                    top-level %include (pulls in const + types)
│   │   ├── tpe.const.klt              constants: error codes, limits, AR[] types
│   │   ├── tpe.types.klt              TPEPROGRM struct definition
│   │   ├── tpe.vars.klh               %import aliases for AR[] getters
│   │   ├── hash_tpeprog.klt           hash config for open-program tracking
│   │   └── tpefile.friend.member.klt  friend member declarations
│   └── tpepose/
│       ├── tpepose.klc                class template — all tpepose__ implementations (689 lines)
│       ├── tpepose.klh                declare_member entries for tpepose class
│       ├── tpepose.const.klt          MOVE_JOINT/LINEAR/CIRCLE/ARC enums
│       ├── tpepose.general.type.klt   t_POSEDATA, t_MOTIONCALL, t_MOTIONSTAT structs
│       └── tpepose.system.type.klt    t_POSITION, t_JPOSITION (system-dependent types)
├── tp/
│   ├── tpe_exists.kl                  TP-callable: checks if a .TP file exists
│   └── tpe_name.kl                    TP-callable: returns program name string
└── test/
    ├── test_tpe.kl                    KUnit: pose read/copy/compose tests
    ├── test_tpe_wrt.kl                KUnit: TP program create/write tests
    ├── test_tpe_ls.tpp                TP Plus: calls Karel test runner
    ├── test_tpe_pos.tpp               TP Plus: calls pose test runner
    └── configs/
        ├── defaultrotpose.klt         dual-group robot config (RBT_GRP=1, ROT_GRP=2)
        └── rotary.motion.impl.klt     rotary group motion interface config
```

---

## Full API Reference

### Constants (`tpe.const.klt`)

```
Type_INTEGER = 1         GET_TPE_PRM data type code for integer
Type_REAL    = 2         GET_TPE_PRM data type code for real
Type_STRING  = 3         GET_TPE_PRM data type code for string

NOPARAM     = 17042      FANUC: AR[] index does not exist
INVALIDTYPE = 17043      FANUC: wrong data type requested
TPE_NOPARAM = 17042      duplicate alias
TPE_P_UNINIT = 17038     FANUC: P[] position uninitialized

MAX_POINTS  = 3000       upper bound when scanning for last pose index
TPE_IS_CLOSE = 99999     sentinel: program not in hash (closed)
TPE_IS_OPEN  = 80000     sentinel: program open and tracked
```

### Motion Type Constants (`tpepose.const.klt`)

```
MOVE_JOINT  = 1    J  motion statement
MOVE_LINEAR = 2    L  motion statement
MOVE_CIRCLE = 3    C  motion statement
MOVE_ARC    = 4    A  motion statement
```

### Types

#### `TPEPROGRM` (`tpe.types.klt`)
Internal hash value. Stores the OPEN_TPE handle.
```
TPEPROGRM = STRUCTURE
  open_id : INTEGER    handle returned by OPEN_TPE()
  status  : INTEGER    TPE_IS_OPEN (80000) or error code
ENDSTRUCTURE
```

#### `t_POSEDATA` (`tpepose.general.type.klt`)
Generic container for either cartesian or joint pose data.
```
t_POSEDATA = STRUCTURE
  xyz   : XYZWPREXT   cartesian pose (extended for multi-group)
  joint : JOINTPOS    joint angles
  tool  : SHORT       tool frame number
  frame : SHORT       user frame number
ENDSTRUCTURE
```

#### `t_MOTIONCALL` (`tpepose.general.type.klt`)
TA/TB callback data parsed from motion statement modifiers.
```
t_MOTIONCALL = STRUCTURE
  prog        : STRING[16]  callback program name
  callTime    : REAL        timing value
  pin         : INTEGER     DO pin number
  pinState    : BOOLEAN     ON or OFF
  timeBefore  : BOOLEAN     TRUE=TA (before), FALSE=TB (after)
ENDSTRUCTURE
```

#### `t_MOTIONSTAT` (`tpepose.general.type.klt`)
Fully parsed motion statement from an LS file line.
```
t_MOTIONSTAT = STRUCTURE
  callback     : t_MOTIONCALL
  id           : SHORT    P[n] index
  typ          : BYTE     MOVE_JOINT / MOVE_LINEAR / MOVE_CIRCLE / MOVE_ARC
  term         : BYTE     0 = FINE, else CNT coefficient
  speed        : BYTE     speed percentage or speed code
  accel        : BYTE     ACC value
  tool_offset  : BYTE     tool offset register number
  frame_offset : BYTE     frame offset register number
  coord        : BOOLEAN  coordinated motion (COO)
  rtcp         : BOOLEAN  Robot Tool Center Point
  fine         : BOOLEAN  FINE termination
ENDSTRUCTURE
```

#### `t_POSITION` (`tpepose.system.type.klt`)
Cartesian pose storage with tool/frame metadata.
```
t_POSITION = STRUCTURE
  id    : SHORT
  tool  : BYTE
  frame : BYTE
  pose  : MOTION_DATA_TYPE    system-dependent (XYZWPR or XYZWPREXT)
ENDSTRUCTURE
```

#### `t_JPOSITION` (`tpepose.system.type.klt`)
Joint pose storage with tool/frame metadata.
```
t_JPOSITION = STRUCTURE
  id    : SHORT
  tool  : BYTE
  frame : BYTE
  pose  : MOTION_DATA_JOINT_TYPE    system-dependent (JOINTPOS or JOINTPOS6)
ENDSTRUCTURE
```

---

### tpefile Public Routines (`tpe.klh`)

All prefixed `tpe__`.

#### Lifecycle / Handle Management

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `tpe__constructor` | `()` | Clears the internal hash table tracking open programs. Call once at startup. |
| `tpe__destructor` | `()` | Same as constructor — clears hash. Call at shutdown. |
| `tpe__open` | `(TPE_name : STRING)` | `OPEN_TPE(RW access)`, stores open_id in hash. Aborts on error. |
| `tpe__close` | `(TPE_name : STRING)` | `CLOSE_TPE`, removes from hash. Warns if not open. |
| `tpe__get_open_id` | `(TPE_name : STRING) : TPEPROGRM` | Returns stored `{open_id, status}` from hash. Warns if not open. |

#### LS File I/O

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `tpe__open_file` | `(TPE_name : STRING)` | Opens `MD:/NAME.LS` in RO mode and reads forward until `/MN` marker. Sets global `curr_prog`, `curr_line`. |
| `tpe__close_file` | `()` | Closes `tpefile` handle, clears `curr_prog`/`curr_line`. |
| `tpe__sanitize_line` | `()` | Strips leading spaces, leading line-number+colon prefix, trailing spaces, and trailing `;` from global `curr_line`. |
| `tpe__create_ls` | `(name : STRING)` | Opens `MD:/NAME.LS` in RW mode, writes `/PROG NAME` header. Sets `new_prog`, `newfile` globals. |
| `tpe__close_ls` | `()` | Closes `newfile` handle. |
| `tpe__copy_header_ls` | `(TPE_Name : STRING)` | Reads from an existing LS file up to `/MN`, writes header section to `new_prog`. Used when cloning a program and replacing only motion statements. |
| `tpe__write_end` | `()` | Writes `/POS` then `/END` to `new_prog`. Finalizes LS structure. |
| `tpe__write_pause` | `()` | Appends `:  PAUSE ;` line to `new_prog`. |
| `tpe__write_uframe` | `(num : INTEGER)` | Appends `:  UFRAME_NUM=<num> ;` to `new_prog`. |
| `tpe__write_utool` | `(num : INTEGER)` | Appends `:  UTOOL_NUM=<num> ;` to `new_prog`. |
| `tpe__write_line` | `(str : STRING)` | Writes raw string to `new_prog`. For motion statements or arbitrary TP lines. |

#### TP Program Creation & Copying

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `tpe__create` | `(name : STRING)` | `CREATE_TPE`. If program already exists, deletes the old `.LS` file first, then recreates. |
| `tpe__copy` | `(TPE_name, new_name : STRING)` | `COPY_TPE(src, dst, TRUE)`. Requires `tpe__open` first. |
| `tpe__ls_exists` | `(TPE_Name : STRING) : BOOLEAN` | Opens `MD:/NAME.TP` in RO mode, checks `IO_STATUS`. Returns TRUE if file accessible. |
| `tpe__pns_name` | `(prefix : STRING; number : INTEGER) : STRING` | Concatenates `prefix + number` → `"P1"`, `"P2"`, etc. |

#### AR[] Parameter Reading

These call the FANUC built-in `GET_TPE_PRM(reg_no, data_type, ...)`. They are called from `tp/*.kl` programs that are invoked by TP programs.

| Routine | Signature | Notes |
|---------|-----------|-------|
| `tpe__get_int_arg` | `(reg_no : INTEGER) : INTEGER` | Aborts if wrong type. |
| `tpe__get_real_arg` | `(reg_no : INTEGER) : REAL` | Promotes int→real if needed. Aborts if string. |
| `tpe__get_string_arg` | `(reg_no : INTEGER) : STRING` | Returns `STRING[32]`. Aborts if wrong type. |
| `tpe__get_boolean_arg` | `(reg_no : INTEGER) : BOOLEAN` | Reads int AR[], returns `int_value <> 0`. |
| `tpe__get_vector_arg` | `(reg_no : INTEGER) : VECTOR` | Reads `PR[reg_no]` via `GET_POS_REG`, extracts `x,y,z` only. |
| `tpe__parameter_exists` | `(reg_no : INTEGER) : BOOLEAN` | No abort — returns FALSE if `NOPARAM`, TRUE otherwise. |

---

### tpepose Class Methods (`tpepose.klh`)

Instantiated via `%class <classname>('tpepose.klc', 'tpepose.klh', <config.klt>)`. Config `.klt` defines `RBT_GRP`, `ROT_GRP`, `MOTION_DATA_TYPE`, `MOTION_DATA_JOINT_TYPE`, and the `motion_type()` macro for dual-group support.

All prefixed `<classname>__`.

#### Motion Statement Utilities

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `set_motion_default` | `() : t_MOTIONSTAT` | Returns zero-filled `t_MOTIONSTAT` with safe defaults (fine=FALSE, accel=0, etc.). |
| `is_line_a_motion` | `(motion_line : STRING) : BOOLEAN` | Returns TRUE if first char is `J`, `L`, `C`, or `A`. |
| `parse_motion` | `(motion_line : STRING) : t_MOTIONSTAT` | Full LS motion line parser. Extracts type, P[] index, speed, termination (FINE/CNT), ACC, COO, RTCP, tool/frame offsets, TA/TB callbacks. |

#### Pose Getters

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `get_pose_grp` | `(TPE_name : STRING; pose_no, grp_no : INTEGER; pose : t_POSEDATA) : BOOLEAN` | Low-level: reads one P[] from open TP program, auto-detects cart vs joint. Returns FALSE if uninitialized. |
| `get_cart_pose` | `(TPE_name : STRING; pose_no : INTEGER) : MOTION_DATA_TYPE` | Returns cartesian pose from P[pose_no]. Requires `tpe__open` first. |
| `get_joint_pose` | `(TPE_name : STRING; pose_no : INTEGER) : MOTION_DATA_JOINT_TYPE` | Returns joint pose from P[pose_no]. |
| `get_last_pose_index` | `(TPE_name : STRING) : INTEGER` | Scans P[] up to `AVL_POS_NUM + 20` to find highest occupied index. |
| `get_cart_poses` | `(TPE_name : STRING; poses : PATH OF t_POSITION)` | Reads all cartesian P[] into PATH, including tool/frame for each. |
| `get_joint_poses` | `(TPE_name : STRING; poses : PATH OF t_JPOSITION)` | Reads all joint P[] into PATH. |

#### Pose Setters

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `set_cart_pose` | `(TPE_name : STRING; pose_no : INTEGER; pose : MOTION_DATA_TYPE)` | `SET_POS_TPE(open_id, pose_no, pose, ...)`. Handles dual-group via config macros. |
| `set_joint_pose` | `(TPE_name : STRING; pose_no : INTEGER; pose : MOTION_DATA_JOINT_TYPE)` | `SET_JPOS_TPE(open_id, pose_no, pose, ...)`. |
| `set_tpe_poses` | `(TPE_name : STRING; cart_poses, joint_poses : PATH)` | Iterates both PATHs, writing each pose using the `id` field from `t_POSITION`/`t_JPOSITION`. |
| `set_tpe_groups` | `(name : STRING)` | Calls `SET_ATTR_PRG` to configure group mask on a TP program. |

#### Pose Copy & Composition

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `copy_cart_poses` | `(poses, out_poses : PATH OF t_POSITION)` | Deep copy via `MOTION_DATA_FILE__copy` per node. |
| `copy_joint_poses` | `(poses, out_poses : PATH OF t_JPOSITION)` | Deep copy of joint poses. |
| `multi_cart_poses` | `(poses, poses2 : PATH OF t_POSITION)` | Element-wise pose composition `poses[i] = poses[i] * poses2[i]` via `MOTION_DATA_FILE__poseMul`. PATH lengths must match. Aborts otherwise. |

#### Motion Statement Collection

| Routine | Signature | What it does |
|---------|-----------|--------------|
| `get_motion_statements` | `(pth : PATH OF t_MOTIONSTAT)` | Reads from global `curr_line` until `/POS` marker. Calls `is_line_a_motion` + `parse_motion` per line. Appends results to PATH. Requires `tpe__open_file` first. |

---

## Core Patterns

### Pattern 1: AR[] Argument Dispatch (TP → Karel Bridge)

All `tp/*.kl` programs follow this pattern. The TP program calls Karel with `CALL karelprogram(AR[1]=funccode, AR[2]=param, ...)`.

```karel
-- tp/my_interface.kl
PROGRAM my_interface
VAR
  func_ : INTEGER
  param1 : STRING[32]
  out_reg : INTEGER

%from tpe.vars.klh %import get_int_arg, get_string_arg
%from registers.klh %import set_boolean

BEGIN
  func_   = tpe__get_int_arg(1)
  param1  = tpe__get_string_arg(2)
  out_reg = tpe__get_int_arg(3)

  SELECT func_ OF
    CASE(1): registers__set_boolean(out_reg, do_something(param1))
    CASE(2): do_other_thing(param1)
  ENDSELECT
END my_interface
```

Real usage: `lib/forms/tp/delete_form.kl`, `lib/shapes/tp/shp_bisecv.kl`, `lib/paths/pathgen/lib/pathgen.klc`.

### Pattern 2: Read All Poses from a TP Program

```karel
-- Requires: %include tpepose.general.type.klt, tpepose.system.type.klt
-- Requires: %class tsttpe(tpepose.klc, tpepose.klh, myconfig.klt)

VAR
  cart_poses : PATH nodedata = t_POSITION
  jnt_poses  : PATH nodedata = t_JPOSITION

BEGIN
  tpe__open(MY_PROGRAM)
  tsttpe__get_cart_poses(MY_PROGRAM, cart_poses)
  tsttpe__get_joint_poses(MY_PROGRAM, jnt_poses)
  tpe__close(MY_PROGRAM)
END
```

Real usage: `lib/TPE/test/test_tpe.kl` lines 134–167.

### Pattern 3: Clone a TP Program and Write Modified Poses

```karel
BEGIN
  tpe__open(SOURCE_PROG)
  tsttpe__get_cart_poses(SOURCE_PROG, cart_poses)
  tpe__close(SOURCE_PROG)

  -- Apply offsets via pose composition
  tsttpe__copy_cart_poses(cart_poses, offset_poses)
  buildOffsets(offset_poses)           -- user routine fills t_POSITION PATH
  tsttpe__multi_cart_poses(cart_poses, offset_poses)

  -- Write into a new copy
  tpe__copy(SOURCE_PROG, DEST_PROG)
  tpe__open(DEST_PROG)
  tsttpe__set_tpe_poses(DEST_PROG, cart_poses, jnt_poses)
  tpe__close(DEST_PROG)
END
```

Real usage: `lib/TPE/test/test_tpe.kl` `t_gettpepos` routine.

### Pattern 4: Create a New TP Program from Karel (LS Write)

```karel
BEGIN
  tpe__create('MYPROGRAM')       -- CREATE_TPE; delete old if exists
  tpe__create_ls('MYPROGRAM')    -- open MD:/MYPROGRAM.LS for writing

  tpe__copy_header_ls('TEMPLATE_PROG')  -- copy header from template (optional)
  tpe__write_uframe(1)
  tpe__write_utool(1)
  tpe__write_line(':  J P[1] 50% FINE ;')
  tpe__write_line(':  L P[2] 200mm/sec CNT100 ;')
  tpe__write_pause

  tpe__write_end           -- writes /POS and /END
  tpe__close_ls
END
```

Real usage: `lib/TPE/test/test_tpe_wrt.kl`.

### Pattern 5: Parse Motion Statements from LS File

```karel
VAR
  motions : PATH nodedata = t_MOTIONSTAT
  ms : t_MOTIONSTAT

BEGIN
  tpe__open_file('MYPROGRAM')     -- opens .LS and seeks to /MN
  tsttpe__get_motion_statements(motions)
  tpe__close_file

  -- Iterate parsed statements
  FOR i = 1 TO PATH_LEN(motions) DO
    ms = NODE_DATA(motions[i])
    IF ms.typ = MOVE_LINEAR THEN
      -- process linear moves
    ENDIF
  ENDFOR
END
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Calling `tpe__get_*_arg` outside a TP-invoked Karel program | `NOPARAM` (17042) or garbage data | These only work when called from a Karel program that was invoked by a TP program with AR[] arguments. |
| Calling `set_cart_pose` / `get_cart_pose` without `tpe__open` first | `FILE_NOT_OPEN` warning, then likely FANUC built-in error | Always `tpe__open(name)` before any `GET_POS_TPE` / `SET_POS_TPE` calls. |
| Using `tpe__open_file` without `tpe__close_file` after | File handle leak; next open of same file fails | Always pair them; `tpe__close_file` is a distinct call from `tpe__close`. |
| Passing `reg_no = 0` to `tpe__get_vector_arg` | `TPERROR` print + possible abort | Validate that position register index > 0 before calling. |
| `multi_cart_poses` with mismatched PATH lengths | `ABORT` with `ARR_LEN_MISMATCH` error | Ensure both paths have the same length; use `copy_cart_poses` to clone before populating offsets. |
| Forgetting `tpe__write_end` when creating an LS file | Incomplete LS file; FANUC loader rejects program | Always call `tpe__write_end` before `tpe__close_ls`. |
| Using `tpe__copy` without an open handle | Warning + no copy | `tpe__copy` calls `COPY_TPE` directly but does not check if the source is open via the hash. Open it first anyway for safety. |

---

## Dependencies

### This module depends on:
- `pose` — `jpos_to_jpos6`, `jpos_to_jpos2`, pose arithmetic in tpepose
- `errors` — `CHK_STAT`, `karelError`, named error codes
- `files` — `files__open`, `files__close`, `files__read_line_ref`, `files__write_line`
- `Strings` — `lstrip`, `rstrip`, `char_index`, `takeStr`, `takeNextStr`, `s_to_i`, `s_to_r`, `i_to_s`, `getIntInStr`, `to_upper`
- `Hash` — hash table for open-program tracking (`hashname` class)
- `pathmotion`, `pathlib`, `pathmake` — used by tpepose configs for dual-group robots
- `ktransw-macros` — `funcname`, `declare_function`, `declare_member`, `header_guard`

### Modules that depend on this module:
- `registers` / `regmap.klc` — reads AR[] via `tpe__get_int_arg`, `tpe__get_string_arg`
- `forms` / `forms_` — `load_form.kl`, `delete_form.kl` use AR[] getters
- `paths/pathgen` — function dispatch via `tpe__get_int_arg`
- `shapes` — `shp_*.kl` TP bridge programs use `tpe__get_int_arg`, `tpe__get_vector_arg`
- `iterator` — TP interface dispatches via AR[] getters
- `draw` — canvas/polygon TP interfaces use AR[] getters

---

## Build / Integration Notes

- **rossum project name:** `TPElib` — other modules must list `"TPElib"` in their `depends` array.
- **includes path:** rossum adds `lib/tpefile` and `lib/tpepose` to the include path. Use `%from tpe.klh %import` and `%from tpe.vars.klh %import` to selectively pull routines.
- **tpe.vars.klh** provides short `%import` aliases (e.g. `get_int_arg`, `get_string_arg`) for use inside TP bridge programs. Preferred over importing the full `tpe.klh`.
- **tpepose is a class template:** instantiate it with `%class <name>('tpepose.klc', 'tpepose.klh', '<config.klt>')`. The config must define `RBT_GRP`, `ROT_GRP`, `MOTION_DATA_TYPE`, `MOTION_DATA_JOINT_TYPE`, and the `motion_type(grp)` macro.
- **Dual-group support:** The `tpepose.klc` template uses `#ifdef ROT_GRP` guards. For single-group robots, set `ROT_GRP` to the same value as `RBT_GRP` or use the default single-group config.
- **Global state in tpelib.kl:** `curr_prog`, `curr_line`, `tpefile`, `new_prog`, `newfile` are module-level globals. Only one LS file can be open for reading and one for writing simultaneously.
- **TP interface registration:** `tpe_exists` is the only routine registered in `tp-interfaces` in `package.json`. Other TP bridge programs in dependent modules (`lib/shapes/tp/`, `lib/forms/tp/`, etc.) are compiled separately and use `%from tpe.vars.klh %import` to access the AR[] getters.
