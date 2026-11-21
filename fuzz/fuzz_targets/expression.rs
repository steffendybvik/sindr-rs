//! Arbitrary text into the behavioural-expression parser. Nothing may
//! panic or overflow the stack: bad text is an `ExpressionError`. An
//! expression that parses must print as text that parses back to an equal
//! expression, and evaluate without panicking.

#![no_main]

use libfuzzer_sys::fuzz_target;
use sindr::expression::Expression;

fuzz_target!(|data: &[u8]| {
    let Ok(text) = std::str::from_utf8(data) else {
        return;
    };
    let Ok(expr) = Expression::parse(text) else {
        return;
    };
    let printed = expr.to_string();
    match Expression::parse(&printed) {
        Ok(back) => assert_eq!(back, expr, "`{text}` printed as `{printed}`"),
        Err(err) => panic!("`{text}` printed as `{printed}`, which doesn't parse: {err}"),
    }
    let values: Vec<f64> = (0..expr.variables().len()).map(|k| k as f64 - 1.5).collect();
    let _ = expr.evaluate(&values);
});
