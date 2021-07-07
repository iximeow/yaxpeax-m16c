use yaxpeax_arch::{Decoder, LengthedInstruction, U8Reader};
use yaxpeax_m16c::InstDecoder;

use std::fmt::Write;

fn test_display(data: &[u8], expected: &'static str) {
    test_display_under(&InstDecoder::default(), data, expected);
}

fn test_invalid(data: &[u8]) {
    let decoder = InstDecoder::default();
    let mut reader = U8Reader::new(data);
    match decoder.decode(&mut reader) {
        Ok(instr) => {
            assert!(false, "incorrectly decoded {:?} from {:02x?} under decoder {}", instr, data, decoder);
        },
        Err(_err) => {
            // we expect an error here. one day, specify what kind of error was expected maybe...
        }
    }

}

fn test_display_under(decoder: &InstDecoder, data: &[u8], expected: &'static str) {
    let mut hex = String::new();
    for b in data {
        write!(hex, "{:02x}", b).unwrap();
    }
    let mut reader = U8Reader::new(data);
    match decoder.decode(&mut reader) {
        Ok(instr) => {
            let text = format!("{}", instr);
            assert!(
                text == expected,
                "display error for {}:\n  decoded: {:?} under decoder {}\n displayed: {}\n expected: {}\n",
                hex,
                instr,
                decoder,
                text,
                expected
            );
            // while we're at it, test that the instruction is as long, and no longer, than its
            // input
            assert_eq!(instr.len().to_const() as usize, data.len(), "instruction length is incorrect, wanted instruction {}", expected);
        },
        Err(e) => {
            assert!(false, "decode error ({}) for {} under decoder {}:\n  expected: {}\n", e, hex, decoder, expected);
        }
    }
}

#[test]
fn test() {
    // from whitequark's screenshot in https://github.com/whitequark/binja-m16c
    test_display(&[0x7c, 0xf2, 0x03], "enter #03h");
    test_display(&[0x73, 0x1b, 0xfd], "mov.w r1, -3[fb]");
    test_display(&[0xb6, 0xff], "mov.b #00h, -1[fb]");
    test_display(&[0xe6, 0x4f, 0xff], "cmp.b #4fh, -1[fb]");
    test_display(&[0x68, 0x0c], "jgeu $+12");
    test_display(&[0x7d, 0xf2], "exitd");
    test_display(&[0x32, 0xff], "mov.b -1[fb], a0");
    test_display(&[0xa1, 0xb4, 0xfd], "add.w -3[fb], a0");
    test_display(&[0xd8, 0xf6], "mov.b #-01h, [a0]");
    test_display(&[0xa6, 0xff], "inc.b -1[fb]");
    test_display(&[0xfe, 0xf1], "jmp.b $-15");

    // these are from the listsing in
    // https://renesasrulz.com/other_products/m16c/f/m16c---forum/1375/how-to-recognize-memory-size-of-m16c-62p
    test_display(&[0x5e, 0x01], "btst #06h, 1[sb]"); // offset might be wrong
    test_display(&[0xc7, 0xff, 0xb4, 0x03], "mov.b #-01h, [#3b4h]");
    test_display(&[0xe7, 0x00, 0xb4, 0x03], "cmp.b #00h, [#3b4h]");
    test_display(&[0x7e, 0x2a, 0x0e, 0x02], "bmeq 6,1[sb]"); // offset might be wrong
    test_display(&[0x68, 0x28], "jgeu $+40"); // "s_program_m0" in post, looks like .. 7?
    test_display(&[0x75, 0xc2, 0x41, 0x00], "mov.w #41h, r2");
    test_display(&[0xf5, 0x71, 0x03], "jsr.w $+881"); // s_command_write
    test_display(&[0x75, 0xc2, 0x40, 0x00], "mov.w #40h, r2");
    test_display(&[0xf5, 0x4a, 0x03], "jsr.w $+842"); // s_command_write

    // not totally sure, just ensuring consistency with my read of the docs
    test_display(&[0x7e, 0x28, 0x0e, 0x02], "bmeq 14[a0]"); // 14[a0] means an index wildly different from 6,1[sb] - it is bit `a0` offset from byte 14, whereas `6,1[sb]` is bit 6 in byte `1+sb`
    test_display(&[0x7d, 0x10], "jsri r2r0");
    test_display(&[0x09, 0x08], "mov.b 8[sb], r0l");
    test_display(&[0x7d, 0xeb, 0x44, 0x66], "add.w #6644h, sp");
    test_display(&[0x7c, 0xeb, 0x54], "add.b #54h, sp");
    test_invalid(&[0x7d, 0x02]);
    test_invalid(&[0x7d, 0x03]);
    test_invalid(&[0x7d, 0x05]);
    test_display(&[0xe7, 0xf7, 0x12, 0x42], "cmp.b #-09h, [#4212h]");
}
