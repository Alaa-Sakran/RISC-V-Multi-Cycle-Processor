.data
data: .word 0x12345678  

.text
.globl _start
_start:
    # Initialize registers
    li x1, 10     # x1 = 10
    li x2, 20     # x2 = 20
    li x3, 30     # x3 = 30
    li x4, 40     # x4 = 40
    li x5, 0      # x5 = 0 (initially)

    # R-type instructions
    add x5, x1, x2   # x5 = x1 + x2 = 30
    sub x6, x3, x1   # x6 = x3 - x1 = 20
    sll x7, x1, x2   # x7 = x1 << (x2 & 31)
    sra x8, x4, x1   # x8 = x4 >> (x1 & 31)
    and x9, x1, x2   # x9 = x1 & x2
    or x10, x1, x2   # x10 = x1 | x2
    xor x11, x1, x2  # x11 = x1 ^ x2
    slt x12, x1, x2  # x12 = (x1 < x2) ? 1 : 0

    # I-type instructions
    addi x13, x1, 5   # x13 = x1 + 5
    slli x14, x1, 3   # x14 = x1 << 3
    srli x15, x1, 2   # x15 = x1 >> 2
    ori x16, x1, 0xFF # x16 = x1 | 0xFF
    xori x17, x1, 0xF # x17 = x1 ^ 0xF

    # lw and sw instructions
    lui x18, %hi(data) # Load upper immediate of data address
    addi x18, x18, %lo(data) # Load lower immediate of data address
    lw x19, 0(x18)  # Load word from memory to x19
    sw x19, 4(x18)  # Store word from x19 to memory

    # Branch instructions
    beq x1, x1, equal      # Branch if x1 == x1 (always)
    blt x1, x2, less_than  # Branch if x1 < x2

equal:
    addi x5, x5, 1  # Dummy instruction to check beq

less_than:
    addi x5, x5, 2  # Dummy instruction to check blt

    # jal and jalr instructions
    jal x20, jump_label   # Jump and link to jump_label
    jalr x21, 0(x20)      # Jump and link register to address in x20

jump_label:
    addi x22, x0, 42  # Dummy instruction after jump
