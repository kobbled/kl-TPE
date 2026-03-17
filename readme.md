# TPE

Karel library for interacting with FANUC TP (Teach Pendant) programs at runtime — reading and writing position registers, parsing LS motion statements, creating TP programs from Karel code, and bridging AR[] arguments from TP callers into Karel routines.

## Overview

The TPE module solves two distinct problems:

**1. TP → Karel argument passing.** When a TP program calls a Karel subroutine, it passes parameters via `AR[]` registers. TPE wraps the `GET_TPE_PRM` built-in into typed getters (`tpe__get_int_arg`, `tpe__get_string_arg`, etc.) that do type validation and abort cleanly on failure. Every Ka-Boost module that exposes a TP interface uses these.

**2. Karel → TP program manipulation.** The `tpepose` class provides OOP-style read/write access to the P[] position table inside an open TP program — bulk-read all poses, compose offsets, write them back. It also parses LS ASCII motion statements into structured `t_MOTIONSTAT` records, which is used by the `paths` pipeline to programmatically generate and modify robot motion programs.

---

## Files

| File | Purpose |
|------|---------|
| `lib/tpefile/tpelib.kl` | All `tpe__` routine implementations (442 lines) |
| `lib/tpefile/tpe.klh` | Public header — `declare_function` entries, include this to import routines |
| `lib/tpefile/tpe.vars.klh` | Short `%import` aliases for AR[] getters — use in TP bridge programs |
| `lib/tpefile/tpe.klt` | Top-level `%include` that pulls in constants and types |
| `lib/tpefile/tpe.const.klt` | Constants: error codes, `MAX_POINTS`, `Type_INTEGER/REAL/STRING` |
| `lib/tpefile/tpe.types.klt` | `TPEPROGRM` struct definition |
| `lib/tpefile/hash_tpeprog.klt` | Hash table config for tracking open program handles |
| `lib/tpefile/tpefile.friend.member.klt` | Friend member declarations for cross-class access |
| `lib/tpepose/tpepose.klc` | `tpepose` class template (689 lines) |
| `lib/tpepose/tpepose.klh` | `declare_member` entries — include when instantiating the class |
| `lib/tpepose/tpepose.const.klt` | `MOVE_JOINT`, `MOVE_LINEAR`, `MOVE_CIRCLE`, `MOVE_ARC` |
| `lib/tpepose/tpepose.general.type.klt` | `t_POSEDATA`, `t_MOTIONCALL`, `t_MOTIONSTAT` structs |
| `lib/tpepose/tpepose.system.type.klt` | `t_POSITION`, `t_JPOSITION` (system-dependent pose wrappers) |
| `tp/tpe_exists.kl` | TP-callable program: returns BOOLEAN indicating if a `.TP` file exists |
| `tp/tpe_name.kl` | TP-callable program: returns the program name string |
| `test/test_tpe.kl` | KUnit tests: pose read, copy, compose, write-back |
| `test/test_tpe_wrt.kl` | KUnit tests: TP program create and LS write |

---

## API Reference

### Types

#### `TPEPROGRM`
Returned by `tpe__get_open_id`. Holds the handle from `OPEN_TPE`.
```
TPEPROGRM = STRUCTURE
  open_id : INTEGER    -- handle to pass to FANUC built-ins
  status  : INTEGER    -- TPE_IS_OPEN (80000) or error code
ENDSTRUCTURE
```

#### `t_MOTIONSTAT`
Fully parsed motion statement from an LS file line.
```
t_MOTIONSTAT = STRUCTURE
  callback     : t_MOTIONCALL  -- TA/TB timing and DO output
  id           : SHORT         -- P[n] index
  typ          : BYTE          -- MOVE_JOINT/LINEAR/CIRCLE/ARC
  term         : BYTE          -- 0 = FINE, else CNT coefficient
  speed        : BYTE          -- speed percentage or code
  accel        : BYTE          -- ACC value
  tool_offset  : BYTE          -- tool offset register number
  frame_offset : BYTE          -- frame offset register number
  coord        : BOOLEAN       -- coordinated motion (COO)
  rtcp         : BOOLEAN       -- Robot Tool Center Point
  fine         : BOOLEAN       -- FINE termination flag
ENDSTRUCTURE
```

#### `t_POSEDATA`
Generic container for either cartesian or joint pose.
```
t_POSEDATA = STRUCTURE
  xyz   : XYZWPREXT   -- cartesian pose
  joint : JOINTPOS    -- joint angles
  tool  : SHORT       -- tool frame number
  frame : SHORT       -- user frame number
ENDSTRUCTURE
```

#### `t_POSITION` / `t_JPOSITION`
Pose storage with identity metadata, used in bulk read/write PATHs.
```
t_POSITION = STRUCTURE          t_JPOSITION = STRUCTURE
  id    : SHORT                   id    : SHORT
  tool  : BYTE                    tool  : BYTE
  frame : BYTE                    frame : BYTE
  pose  : MOTION_DATA_TYPE        pose  : MOTION_DATA_JOINT_TYPE
ENDSTRUCTURE                    ENDSTRUCTURE
```

---

### tpefile Routines

#### Program Lifecycle

```
tpe__constructor()
  -- Initialize internal hash table. Call once before any tpe__ usage.

tpe__destructor()
  -- Clear the hash table.

tpe__open(TPE_name : STRING)
  -- Open TP program via OPEN_TPE() (read/write access).
  -- Stores handle in hash. Aborts if OPEN_TPE fails.

tpe__close(TPE_name : STRING)
  -- Close open program via CLOSE_TPE(). Removes from hash.
  -- Warns (non-fatal) if program was not open.

tpe__get_open_id(TPE_name : STRING) : TPEPROGRM
  -- Retrieve stored {open_id, status} for an open program.
  -- Warns if program is not in hash.

tpe__create(name : STRING)
  -- CREATE_TPE. If program already exists, deletes the old .LS file first.
  -- Aborts if creation fails.

tpe__copy(TPE_name, new_name : STRING)
  -- COPY_TPE(src, dst, overwrite=TRUE).

tpe__ls_exists(TPE_Name : STRING) : BOOLEAN
  -- Returns TRUE if MD:/NAME.TP exists and is accessible.
```

#### LS File Read/Write

```
tpe__open_file(TPE_name : STRING)
  -- Open MD:/NAME.LS for sequential reading.
  -- Advances past header to /MN (program start marker).
  -- Sets module globals: curr_prog, curr_line.

tpe__close_file()
  -- Close the read-mode file handle. Clears curr_prog/curr_line.

tpe__sanitize_line()
  -- Strips LS line-number prefix, leading/trailing whitespace, and semicolon
  -- from the global curr_line. Call after reading each line.

tpe__create_ls(name : STRING)
  -- Create MD:/NAME.LS for writing. Writes /PROG header.
  -- Sets new_prog, newfile globals.

tpe__close_ls()
  -- Close the write-mode file handle.

tpe__copy_header_ls(TPE_Name : STRING)
  -- Read header of an existing LS file (up to /MN), copy to new_prog.
  -- Use when cloning a program but replacing its motion statements.

tpe__write_end()
  -- Write /POS then /END. Must be the last write before tpe__close_ls.

tpe__write_pause()          -- Appends :  PAUSE ;
tpe__write_uframe(n)        -- Appends :  UFRAME_NUM=n ;
tpe__write_utool(n)         -- Appends :  UTOOL_NUM=n ;
tpe__write_line(str)        -- Writes raw string to new_prog.

tpe__pns_name(prefix, number) : STRING
  -- Returns prefix + number as string. E.g. "P" + 3 → "P3".
```

#### AR[] Parameter Reading

Called from Karel programs that are invoked by TP programs. Each reads `AR[reg_no]` via `GET_TPE_PRM`.

```
tpe__get_int_arg(reg_no : INTEGER) : INTEGER
  -- Reads integer AR[]. Aborts if wrong type.

tpe__get_real_arg(reg_no : INTEGER) : REAL
  -- Reads real AR[]. Promotes integer to real if needed. Aborts if string.

tpe__get_string_arg(reg_no : INTEGER) : STRING
  -- Reads string AR[]. Returns STRING[32]. Aborts if wrong type.

tpe__get_boolean_arg(reg_no : INTEGER) : BOOLEAN
  -- Reads integer AR[]. Returns (value <> 0).

tpe__get_vector_arg(reg_no : INTEGER) : VECTOR
  -- Reads PR[reg_no] via GET_POS_REG, extracts (x, y, z). Aborts if uninit.

tpe__parameter_exists(reg_no : INTEGER) : BOOLEAN
  -- Returns FALSE if AR[reg_no] does not exist. Never aborts.
```

---

### tpepose Class Methods

`tpepose` is a GPP class template. Instantiate it with:

```
%define class_name mytpe
%include myconfig.klt        -- defines RBT_GRP, ROT_GRP, MOTION_DATA_TYPE, etc.
%class mytpe('tpepose.klc', 'tpepose.klh', 'myconfig.klt')
```

All methods below are prefixed with your `class_name__`.

#### Motion Parsing

```
set_motion_default() : t_MOTIONSTAT
  -- Returns zero-filled t_MOTIONSTAT with safe defaults.

is_line_a_motion(motion_line : STRING) : BOOLEAN
  -- TRUE if line starts with J, L, C, or A (TP motion codes).

parse_motion(motion_line : STRING) : t_MOTIONSTAT
  -- Full LS motion line parser.
  -- Extracts: motion type, P[] index, speed, CNT/FINE termination,
  --   ACC value, COO (coordinated), RTCP, tool/frame offsets, TA/TB callbacks.

get_motion_statements(pth : PATH nodedata = t_MOTIONSTAT)
  -- Reads curr_line until /POS marker; appends t_MOTIONSTAT per motion line.
  -- Requires tpe__open_file first.
```

#### Pose Getters

```
get_pose_grp(TPE_name, pose_no, grp_no, pose : t_POSEDATA) : BOOLEAN
  -- Low-level: reads one P[] auto-detecting cart vs joint. Returns FALSE if uninit.

get_cart_pose(TPE_name, pose_no) : MOTION_DATA_TYPE
  -- Cartesian pose from P[pose_no] in open TP program.

get_joint_pose(TPE_name, pose_no) : MOTION_DATA_JOINT_TYPE
  -- Joint pose from P[pose_no].

get_last_pose_index(TPE_name) : INTEGER
  -- Highest occupied P[] index (scans up to AVL_POS_NUM + 20).

get_cart_poses(TPE_name, poses : PATH nodedata = t_POSITION)
  -- Reads all cartesian P[] into PATH, including tool/frame per pose.

get_joint_poses(TPE_name, poses : PATH nodedata = t_JPOSITION)
  -- Reads all joint P[] into PATH.
```

#### Pose Setters

```
set_cart_pose(TPE_name, pose_no, pose : MOTION_DATA_TYPE)
  -- SET_POS_TPE. Handles dual-group via config macros.

set_joint_pose(TPE_name, pose_no, pose : MOTION_DATA_JOINT_TYPE)
  -- SET_JPOS_TPE.

set_tpe_poses(TPE_name, cart_poses, joint_poses : PATH)
  -- Writes all poses using the id field from each t_POSITION / t_JPOSITION node.

set_tpe_groups(name : STRING)
  -- SET_ATTR_PRG: configure group mask on a TP program.
```

#### Pose Copy and Composition

```
copy_cart_poses(poses, out_poses : PATH nodedata = t_POSITION)
  -- Deep copy all cartesian poses into out_poses.

copy_joint_poses(poses, out_poses : PATH nodedata = t_JPOSITION)
  -- Deep copy all joint poses.

multi_cart_poses(poses, poses2 : PATH nodedata = t_POSITION)
  -- Element-wise pose composition: poses[i] = poses[i] * poses2[i].
  -- Both PATHs must have equal length — aborts otherwise.
  -- Result overwrites first PATH.
```

---

## Common Patterns

### Pattern 1: TP → Karel Argument Bridge

Every Ka-Boost TP interface program follows this skeleton. The TP program passes arguments as `AR[1]`, `AR[2]`, etc.

```karel
-- tp/my_function.kl
PROGRAM my_function
VAR
  func_code : INTEGER
  name      : STRING[32]
  out_reg   : INTEGER

%from tpe.vars.klh %import get_int_arg, get_string_arg
%from registers.klh %import set_boolean

BEGIN
  func_code = tpe__get_int_arg(1)
  name      = tpe__get_string_arg(2)
  out_reg   = tpe__get_int_arg(3)

  SELECT func_code OF
    CASE(1): registers__set_boolean(out_reg, do_operation_a(name))
    CASE(2): registers__set_boolean(out_reg, do_operation_b(name))
  ENDSELECT
END my_function
```

Use `tpe.vars.klh` instead of `tpe.klh` — it gives short import aliases and avoids pulling in the full header.

### Pattern 2: Read All Poses from a TP Program

```karel
VAR
  cart_poses : PATH nodedata = t_POSITION
  jnt_poses  : PATH nodedata = t_JPOSITION

BEGIN
  tpe__open('MY_PROG')
  mytpe__get_cart_poses('MY_PROG', cart_poses)
  mytpe__get_joint_poses('MY_PROG', jnt_poses)
  tpe__close('MY_PROG')
  -- cart_poses PATH now holds all P[] in order, with tool/frame data
END
```

### Pattern 3: Copy a TP Program and Modify Its Poses

This is the primary use case in the `paths` pipeline — take an existing TP program, apply a frame offset to all cartesian poses, write into a copy.

```karel
VAR
  cart_poses    : PATH nodedata = t_POSITION
  offset_poses  : PATH nodedata = t_POSITION
  jnt_poses     : PATH nodedata = t_JPOSITION

BEGIN
  -- 1. Read source poses
  tpe__open('SOURCE_PROG')
  mytpe__get_cart_poses('SOURCE_PROG', cart_poses)
  mytpe__get_joint_poses('SOURCE_PROG', jnt_poses)
  tpe__close('SOURCE_PROG')

  -- 2. Build and apply offsets
  mytpe__copy_cart_poses(cart_poses, offset_poses)
  fill_offset_poses(offset_poses)              -- your routine
  mytpe__multi_cart_poses(cart_poses, offset_poses)

  -- 3. Clone program and write modified poses back
  tpe__copy('SOURCE_PROG', 'DEST_PROG')
  tpe__open('DEST_PROG')
  mytpe__set_tpe_poses('DEST_PROG', cart_poses, jnt_poses)
  tpe__close('DEST_PROG')
END
```

### Pattern 4: Create a TP Program from Karel

Karel generates a complete LS file programmatically. The FANUC controller loads `.LS` files as compiled TP programs.

```karel
BEGIN
  tpe__create('NEW_PROG')        -- CREATE_TPE; deletes old if exists
  tpe__create_ls('NEW_PROG')     -- open MD:/NEW_PROG.LS for writing

  tpe__copy_header_ls('TEMPLATE') -- optionally copy header from a template
  tpe__write_uframe(1)
  tpe__write_utool(2)
  tpe__write_line(':  J P[1] 50% FINE ;')
  tpe__write_line(':  L P[2] 200mm/sec CNT100 ;')
  tpe__write_pause

  tpe__write_end    -- writes /POS and /END — required before close
  tpe__close_ls
END
```

### Pattern 5: Parse Motion Statements

```karel
VAR
  motions : PATH nodedata = t_MOTIONSTAT
  ms      : t_MOTIONSTAT

BEGIN
  tpe__open_file('MY_PROG')               -- opens .LS and seeks to /MN
  mytpe__get_motion_statements(motions)   -- fills PATH with parsed t_MOTIONSTAT
  tpe__close_file

  FOR i = 1 TO PATH_LEN(motions) DO
    ms = NODE_DATA(motions[i])
    SELECT ms.typ OF
      CASE(MOVE_LINEAR):
        -- process linear move at P[ms.id]
      CASE(MOVE_JOINT):
        -- process joint move
    ENDSELECT
  ENDFOR
END
```

---

## Common Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Calling `tpe__get_*_arg` when not invoked by a TP program | `NOPARAM` (17042) error, garbage values | These only work inside Karel programs called by TP with `AR[]` arguments |
| Calling `set_cart_pose` / `get_cart_pose` without `tpe__open` first | `FILE_NOT_OPEN` warning, FANUC built-in error | Always `tpe__open(name)` before any pose getter/setter |
| Using `tpe__open_file` without matching `tpe__close_file` | File handle leak; next read fails | Pair every `tpe__open_file` with `tpe__close_file`; note these are separate from `tpe__open` / `tpe__close` |
| Passing `reg_no = 0` to `tpe__get_vector_arg` | `TPERROR` message, possible abort | Validate register index > 0 before calling |
| `multi_cart_poses` with unequal PATH lengths | `ABORT` with `ARR_LEN_MISMATCH` | Lengths must match; use `copy_cart_poses` then populate offsets from the copy |
| Forgetting `tpe__write_end` before `tpe__close_ls` | Truncated LS file rejected by controller | Always write `/POS` + `/END` via `tpe__write_end` as the last step |
| Mixing `tpe__open` / `tpe__close` with `tpe__open_file` / `tpe__close_file` | Confusing — they operate on different handles | `open` / `close` = OPEN_TPE handle for P[] read/write; `open_file` / `close_file` = raw FILE handle for LS ASCII parsing |

---

## Build Flow

TPE compiles as a single Karel program (`tpelib.kl`) that rossum links against `pose`, `errors`, `files`, `Strings`, and `Hash`. The `tpepose` class is expanded at build time by GPP from `tpepose.klc` using your config `.klt`.

```shell
# In your module:
rossum .. -w -o    # generates build.ninja with TPElib in dependency graph
ninja              # compiles tpelib.kl → tpelib.pc
kpush              # deploys to controller

# TPElib is declared as a dependency in your package.json:
# "depends": ["TPElib", ...]
```

See the [Ka-Boost readme](../../readme.md) for full build and deployment instructions.

rossum project name: `TPElib`. Reference it exactly as written in your `package.json` `depends` array.
