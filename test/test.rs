use yaxpeax_arch::{Decoder, LengthedInstruction};
use yaxpeax_m16c::InstDecoder;

use std::fmt::Write;

fn test_display(data: &[u8], expected: &'static str) {
    test_display_under(&InstDecoder::default(), data, expected);
}

fn test_display_under(decoder: &InstDecoder, data: &[u8], expected: &'static str) {
    let mut hex = String::new();
    for b in data {
        write!(hex, "{:02x}", b).unwrap();
    }
    match decoder.decode(data.into_iter().map(|x| *x)) {
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
            assert_eq!(instr.len() as usize, data.len(), "instruction length is incorrect, wanted instruction {}", expected);
        },
        Err(e) => {
            assert!(false, "decode error ({}) for {} under decoder {}:\n  expected: {}\n", e, hex, decoder, expected);
        }
    }
}

#[test]
fn test() {
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
    test_display(&[0x7d, 0xeb, 0x44, 0x66], "add.w #6644h, sp");
    test_display(&[0x7c, 0xeb, 0x54], "add.b #54h, sp");
}
