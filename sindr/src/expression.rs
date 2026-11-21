//! Behavioural expressions: the right-hand side of SPICE's `B` sources,
//! such as `v(a)*v(b)` or `1m*tanh(v(in,ref)/26m)`.
//!
//! An [`Expression`] is parsed from text once and evaluated at every
//! Newton iteration, together with its exact gradient by forward-mode
//! automatic differentiation, so a
//! [`Behavioural`](crate::CircuitElement::Behavioural) source stamps the true
//! Jacobian rather than a finite-difference guess.
//!
//! # Syntax
//!
//! | Form | Meaning |
//! |---|---|
//! | `v(n)`, `v(n,m)` | node voltage, or the voltage of `n` over `m` (V) |
//! | `i(src)` | current through a voltage-defined element (A): an independent or behavioural voltage source, VCVS, CCVS, op-amp output or regulator |
//! | `time` | simulation time (s); 0 outside a transient |
//! | `pi` | π |
//! | `2.5`, `1e-3`, `4.7k`, `10meg`, `26m` | numbers, with SPICE's suffixes (`t g meg k m u n p f mil`) |
//! | `+ - * / ^` (or `**`) | arithmetic; `^` binds tightest and to the right, so `-2^2` is −4 |
//! | `== != < <= > >=`, `&& \|\| !` | comparisons and logic, 1 for true and 0 for false |
//! | `c ? a : b` | `a` where `c` is non-zero, otherwise `b` |
//!
//! Functions: `abs sqrt exp ln log log10 sin cos tan asin acos atan sinh
//! cosh tanh` (one argument; `log` is natural, as in ngspice), `atan2 pow
//! pwr min max` (two), `limit(x, lo, hi)`, and the step-like `u` (1 for
//! `x > 0`), `uramp` (`max(x, 0)`), `sgn`, `floor`, `ceil`. Function and
//! variable names ignore case; node and source names are kept as written.
//!
//! Steps, comparisons and the branches of `?:` have zero derivative, so
//! Newton sees each smooth piece on its own: keep transitions smooth (with
//! `tanh`, say) where convergence matters.
//!
//! ```
//! use sindr::expression::Expression;
//!
//! let e = Expression::parse("v(a) * v(b) + 2m")?;
//! assert_eq!(e.to_string(), "v(a)*v(b)+0.002");
//! assert_eq!(e.evaluate(&[3.0, 4.0]), 12.002);
//! # Ok::<(), sindr::expression::ExpressionError>(())
//! ```

use std::fmt;

/// A parsed behavioural expression. See the [module docs](self) for the
/// syntax.
///
/// It prints as canonical text, which parses back to an equal
/// expression; with the `serde` feature it is serialised as that text.
#[derive(Debug, Clone, PartialEq)]
pub struct Expression {
    root: Node,
    variables: Vec<Variable>,
}

/// A quantity an [`Expression`] reads from the circuit.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Variable {
    /// `v(node)` or `v(node, reference)`: in volts.
    Voltage {
        /// The node.
        node: String,
        /// The node it is measured against; ground when `None`.
        reference: Option<String>,
    },
    /// `i(source)`: the branch current of a voltage-defined element, in
    /// amperes, flowing into its first node.
    Current {
        /// The element's id.
        source: String,
    },
    /// `time`, in seconds.
    Time,
}

/// Why text isn't an [`Expression`].
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{message} at character {position} of `{text}`")]
#[non_exhaustive]
pub struct ExpressionError {
    /// What is wrong.
    pub message: String,
    /// Byte offset into `text` where it was noticed.
    pub position: usize,
    /// The text that was parsed.
    pub text: String,
}

#[derive(Debug, Clone, PartialEq)]
enum Node {
    Num(f64),
    Var(usize),
    Neg(Box<Node>),
    Not(Box<Node>),
    Bin(Op, Box<Node>, Box<Node>),
    Call(Func, Vec<Node>),
    Cond(Box<Node>, Box<Node>, Box<Node>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Op {
    Or,
    And,
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
    Add,
    Sub,
    Mul,
    Div,
    Pow,
}

impl Op {
    fn symbol(self) -> &'static str {
        match self {
            Op::Or => "||",
            Op::And => "&&",
            Op::Eq => "==",
            Op::Ne => "!=",
            Op::Lt => "<",
            Op::Le => "<=",
            Op::Gt => ">",
            Op::Ge => ">=",
            Op::Add => "+",
            Op::Sub => "-",
            Op::Mul => "*",
            Op::Div => "/",
            Op::Pow => "^",
        }
    }

    /// Binding strength: higher binds tighter.
    fn precedence(self) -> u8 {
        match self {
            Op::Or => 2,
            Op::And => 3,
            Op::Eq | Op::Ne | Op::Lt | Op::Le | Op::Gt | Op::Ge => 4,
            Op::Add | Op::Sub => 5,
            Op::Mul | Op::Div => 6,
            Op::Pow => 8,
        }
    }
}

/// Precedence of `?:`, and of the prefix operators, for printing.
const COND: u8 = 1;
const UNARY: u8 = 7;
const ATOM: u8 = 9;

macro_rules! funcs {
    ($($name:ident $text:literal $arity:literal),* $(,)?) => {
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        enum Func { $($name),* }

        impl Func {
            fn from_name(name: &str) -> Option<Func> {
                match name { $($text => Some(Func::$name),)* _ => None }
            }
            fn name(self) -> &'static str {
                match self { $(Func::$name => $text),* }
            }
            fn arity(self) -> usize {
                match self { $(Func::$name => $arity),* }
            }
        }
    };
}

funcs! {
    Abs "abs" 1, Sqrt "sqrt" 1, Exp "exp" 1, Ln "ln" 1, Log "log" 1, Log10 "log10" 1,
    Sin "sin" 1, Cos "cos" 1, Tan "tan" 1, Asin "asin" 1, Acos "acos" 1, Atan "atan" 1,
    Sinh "sinh" 1, Cosh "cosh" 1, Tanh "tanh" 1, U "u" 1, Uramp "uramp" 1, Sgn "sgn" 1,
    Floor "floor" 1, Ceil "ceil" 1, Atan2 "atan2" 2, Pow "pow" 2, Pwr "pwr" 2,
    Min "min" 2, Max "max" 2, Limit "limit" 3,
}

impl Expression {
    /// Parses `text`. See the [module docs](self) for the syntax.
    ///
    /// # Errors
    ///
    /// [`ExpressionError`] for text that isn't an expression, an unknown
    /// function or name, or a function given the wrong number of arguments.
    pub fn parse(text: &str) -> Result<Expression, ExpressionError> {
        Self::parse_with(text, &|_| None)
    }

    /// Parses `text`, reading any other bare name through `constant`
    /// (a SPICE deck's `.param` values).
    pub(crate) fn parse_with(
        text: &str,
        constant: &dyn Fn(&str) -> Option<f64>,
    ) -> Result<Expression, ExpressionError> {
        let mut parser = Parser {
            text,
            pos: 0,
            depth: 0,
            operators: 0,
            variables: Vec::new(),
            constant,
        };
        let root = parser.cond()?;
        parser.skip_ws();
        if parser.pos < text.len() {
            return Err(parser.error("unexpected text"));
        }
        Ok(Expression {
            root,
            variables: parser.variables,
        })
    }

    /// The quantities the expression reads, in the order
    /// [`evaluate`](Self::evaluate) takes their values.
    pub fn variables(&self) -> &[Variable] {
        &self.variables
    }

    /// The expression's value with each of [`variables`](Self::variables)
    /// at the matching entry of `values`. A missing value reads as 0.
    pub fn evaluate(&self, values: &[f64]) -> f64 {
        eval(&self.root, values)
    }

    /// The value, with `gradient[k]` set to its derivative by
    /// `variables()[k]`. `gradient` must be as long as `variables()`.
    pub(crate) fn evaluate_with_gradient(&self, values: &[f64], gradient: &mut [f64]) -> f64 {
        let dual = eval_dual(&self.root, values, self.variables.len());
        gradient.copy_from_slice(&dual.d);
        dual.v
    }

    /// Renames the nodes and sources the expression reads, as flattening a
    /// hierarchy renames nets and elements.
    pub(crate) fn rename(
        &mut self,
        mut node: impl FnMut(&str) -> String,
        mut source: impl FnMut(&str) -> String,
    ) {
        for variable in &mut self.variables {
            match variable {
                Variable::Voltage { node: n, reference } => {
                    *n = node(n);
                    if let Some(r) = reference {
                        *r = node(r);
                    }
                }
                Variable::Current { source: s } => *s = source(s),
                Variable::Time => {}
            }
        }
    }

    /// The nodes the expression reads, ground references included.
    pub(crate) fn nodes(&self) -> impl Iterator<Item = &String> {
        self.variables.iter().flat_map(|v| match v {
            Variable::Voltage { node, reference } => std::iter::once(node)
                .chain(reference.iter())
                .collect::<Vec<_>>(),
            _ => Vec::new(),
        })
    }
}

impl std::str::FromStr for Expression {
    type Err = ExpressionError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        Expression::parse(text)
    }
}

impl fmt::Display for Expression {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write_node(f, &self.root, &self.variables, 0)
    }
}

#[cfg(feature = "serde")]
impl serde::Serialize for Expression {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(self)
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for Expression {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let text = <std::borrow::Cow<'de, str>>::deserialize(deserializer)?;
        Expression::parse(&text).map_err(serde::de::Error::custom)
    }
}

// ------------------------------------------------------------ printing

fn number(f: &mut fmt::Formatter<'_>, value: f64) -> fmt::Result {
    let magnitude = value.abs();
    if value == 0.0 || (1e-3..1e6).contains(&magnitude) {
        write!(f, "{value}")
    } else {
        write!(f, "{value:e}")
    }
}

fn write_node(
    f: &mut fmt::Formatter<'_>,
    node: &Node,
    variables: &[Variable],
    outer: u8,
) -> fmt::Result {
    let own = match node {
        Node::Num(v) if *v < 0.0 => UNARY,
        Node::Num(_) | Node::Var(_) | Node::Call(..) => ATOM,
        Node::Neg(_) | Node::Not(_) => UNARY,
        Node::Bin(op, ..) => op.precedence(),
        Node::Cond(..) => COND,
    };
    let paren = own < outer;
    if paren {
        f.write_str("(")?;
    }
    match node {
        Node::Num(v) => number(f, *v)?,
        Node::Var(k) => match &variables[*k] {
            Variable::Voltage {
                node,
                reference: None,
            } => write!(f, "v({node})")?,
            Variable::Voltage {
                node,
                reference: Some(r),
            } => write!(f, "v({node},{r})")?,
            Variable::Current { source } => write!(f, "i({source})")?,
            Variable::Time => f.write_str("time")?,
        },
        Node::Neg(a) => {
            f.write_str("-")?;
            write_node(f, a, variables, UNARY)?;
        }
        Node::Not(a) => {
            f.write_str("!")?;
            write_node(f, a, variables, UNARY)?;
        }
        Node::Bin(op, a, b) => {
            let p = op.precedence();
            // Left-associative, except `^`: the side that would regroup
            // needs one level more to keep its parentheses.
            let (left, right) = match op {
                Op::Pow => (p + 1, p),
                Op::Eq | Op::Ne | Op::Lt | Op::Le | Op::Gt | Op::Ge => (p + 1, p + 1),
                _ => (p, p + 1),
            };
            write_node(f, a, variables, left)?;
            f.write_str(op.symbol())?;
            write_node(f, b, variables, right)?;
        }
        Node::Call(func, args) => {
            write!(f, "{}(", func.name())?;
            for (k, arg) in args.iter().enumerate() {
                if k > 0 {
                    f.write_str(",")?;
                }
                write_node(f, arg, variables, 0)?;
            }
            f.write_str(")")?;
        }
        Node::Cond(c, a, b) => {
            write_node(f, c, variables, COND + 1)?;
            f.write_str("?")?;
            write_node(f, a, variables, COND)?;
            f.write_str(":")?;
            write_node(f, b, variables, COND)?;
        }
    }
    if paren {
        f.write_str(")")?;
    }
    Ok(())
}

// ------------------------------------------------------------- parsing

/// Deepest nesting of parentheses, calls and prefix operators: past it
/// the parser would overflow the stack on hostile text.
const MAX_DEPTH: usize = 64;

/// Most binary operators in one expression. A chain like `1+1+…` nests
/// one level per operator in the tree, which evaluation recurses through.
const MAX_OPERATORS: usize = 1000;

struct Parser<'a> {
    text: &'a str,
    pos: usize,
    depth: usize,
    operators: usize,
    variables: Vec<Variable>,
    constant: &'a dyn Fn(&str) -> Option<f64>,
}

impl Parser<'_> {
    fn error(&self, message: &str) -> ExpressionError {
        ExpressionError {
            message: message.to_string(),
            position: self.pos,
            text: self.text.to_string(),
        }
    }

    fn rest(&self) -> &str {
        &self.text[self.pos..]
    }

    fn skip_ws(&mut self) {
        let rest = self.rest();
        self.pos += rest.len() - rest.trim_start().len();
    }

    /// Consumes `token` (after whitespace) if it is next.
    fn eat(&mut self, token: &str) -> bool {
        self.skip_ws();
        if self.rest().starts_with(token) {
            self.pos += token.len();
            true
        } else {
            false
        }
    }

    fn expect(&mut self, token: &str) -> Result<(), ExpressionError> {
        if self.eat(token) {
            Ok(())
        } else {
            Err(self.error(&format!("expected `{token}`")))
        }
    }

    fn cond(&mut self) -> Result<Node, ExpressionError> {
        let c = self.binary(Op::Or.precedence())?;
        if self.eat("?") {
            let a = self.cond()?;
            self.expect(":")?;
            let b = self.cond()?;
            return Ok(Node::Cond(Box::new(c), Box::new(a), Box::new(b)));
        }
        Ok(c)
    }

    /// The binary operator next in the text, if it binds at least `min`.
    fn peek_op(&mut self, min: u8) -> Option<(Op, usize)> {
        self.skip_ws();
        let rest = self.rest();
        // Longest spellings first.
        let op = [
            ("||", Op::Or),
            ("&&", Op::And),
            ("==", Op::Eq),
            ("!=", Op::Ne),
            ("<=", Op::Le),
            (">=", Op::Ge),
            ("**", Op::Pow),
            ("<", Op::Lt),
            (">", Op::Gt),
            ("+", Op::Add),
            ("-", Op::Sub),
            ("*", Op::Mul),
            ("/", Op::Div),
            ("^", Op::Pow),
        ]
        .into_iter()
        .find(|(s, _)| rest.starts_with(s))?;
        (op.1.precedence() >= min && op.1 != Op::Pow).then_some((op.1, op.0.len()))
    }

    /// Precedence climbing over the left-associative operators; `^` is
    /// handled under the prefix operators.
    fn binary(&mut self, min: u8) -> Result<Node, ExpressionError> {
        let mut left = self.unary()?;
        while let Some((op, len)) = self.peek_op(min) {
            self.operators += 1;
            if self.operators > MAX_OPERATORS {
                return Err(self.error("too many operators"));
            }
            let p = op.precedence();
            self.pos += len;
            let right = self.binary(p + 1)?;
            left = Node::Bin(op, Box::new(left), Box::new(right));
            if matches!(op, Op::Eq | Op::Ne | Op::Lt | Op::Le | Op::Gt | Op::Ge)
                && self
                    .peek_op(p)
                    .is_some_and(|(next, _)| next.precedence() == p)
            {
                return Err(self.error("comparisons don't chain; add parentheses"));
            }
        }
        Ok(left)
    }

    fn unary(&mut self) -> Result<Node, ExpressionError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return Err(self.error("nested too deeply"));
        }
        let node = self.unary_inner();
        self.depth -= 1;
        node
    }

    fn unary_inner(&mut self) -> Result<Node, ExpressionError> {
        if self.eat("-") {
            return Ok(Node::Neg(Box::new(self.unary()?)));
        }
        if self.eat("+") {
            return self.unary();
        }
        self.skip_ws();
        if self.rest().starts_with('!') && !self.rest().starts_with("!=") {
            self.pos += 1;
            return Ok(Node::Not(Box::new(self.unary()?)));
        }
        let base = self.primary()?;
        if self.eat("**") || self.eat("^") {
            // Right-associative, and above the prefix operators on its
            // right: `2^-1` is 0.5.
            self.operators += 1;
            if self.operators > MAX_OPERATORS {
                return Err(self.error("too many operators"));
            }
            let exponent = self.unary()?;
            return Ok(Node::Bin(Op::Pow, Box::new(base), Box::new(exponent)));
        }
        Ok(base)
    }

    fn primary(&mut self) -> Result<Node, ExpressionError> {
        self.skip_ws();
        if self.eat("(") {
            let inner = self.cond()?;
            self.expect(")")?;
            return Ok(inner);
        }
        let rest = self.rest();
        let first = rest
            .chars()
            .next()
            .ok_or_else(|| self.error("expected a value"))?;
        if first.is_ascii_digit() || first == '.' {
            return self.number();
        }
        if !(first.is_alphabetic() || first == '_') {
            return Err(self.error("expected a value"));
        }
        let start = self.pos;
        let len = rest
            .find(|c: char| !(c.is_alphanumeric() || c == '_'))
            .unwrap_or(rest.len());
        let name = rest[..len].to_ascii_lowercase();
        self.pos += len;
        if self.eat("(") {
            return match name.as_str() {
                "v" => {
                    let node = self.name()?;
                    let reference = if self.eat(",") {
                        Some(self.name()?)
                    } else {
                        None
                    };
                    self.expect(")")?;
                    Ok(self.var(Variable::Voltage { node, reference }))
                }
                "i" => {
                    let source = self.name()?;
                    self.expect(")")?;
                    Ok(self.var(Variable::Current { source }))
                }
                _ => {
                    let func = Func::from_name(&name).ok_or_else(|| {
                        self.pos = start;
                        self.error(&format!("unknown function `{name}`"))
                    })?;
                    let mut args = Vec::new();
                    if !self.eat(")") {
                        loop {
                            args.push(self.cond()?);
                            if self.eat(")") {
                                break;
                            }
                            self.expect(",")?;
                        }
                    }
                    if args.len() != func.arity() {
                        self.pos = start;
                        return Err(self.error(&format!(
                            "`{}` takes {} argument{}, not {}",
                            func.name(),
                            func.arity(),
                            if func.arity() == 1 { "" } else { "s" },
                            args.len()
                        )));
                    }
                    Ok(Node::Call(func, args))
                }
            };
        }
        match name.as_str() {
            "time" => Ok(self.var(Variable::Time)),
            "pi" => Ok(Node::Num(std::f64::consts::PI)),
            _ => match (self.constant)(&name) {
                Some(value) if value.is_finite() => Ok(Node::Num(value)),
                Some(value) => {
                    self.pos = start;
                    Err(self.error(&format!("`{name}` is {value}")))
                }
                None => {
                    self.pos = start;
                    Err(self.error(&format!("unknown name `{name}`")))
                }
            },
        }
    }

    /// A node or source name inside `v(…)` or `i(…)`, kept as written.
    fn name(&mut self) -> Result<String, ExpressionError> {
        self.skip_ws();
        let rest = self.rest();
        let len = rest
            .find(|c: char| {
                !(c.is_alphanumeric() || matches!(c, '_' | '.' | '#' | ':' | '+' | '-'))
            })
            .unwrap_or(rest.len());
        if len == 0 {
            return Err(self.error("expected a node or source name"));
        }
        let name = rest[..len].to_string();
        self.pos += len;
        Ok(name)
    }

    fn var(&mut self, variable: Variable) -> Node {
        let k = match self.variables.iter().position(|v| *v == variable) {
            Some(k) => k,
            None => {
                self.variables.push(variable);
                self.variables.len() - 1
            }
        };
        Node::Var(k)
    }

    fn number(&mut self) -> Result<Node, ExpressionError> {
        let rest = self.rest();
        let bytes = rest.as_bytes();
        let mut end = 0;
        while end < bytes.len() && (bytes[end].is_ascii_digit() || bytes[end] == b'.') {
            end += 1;
        }
        // An exponent, only when digits follow it: `1e3`, not the `e` of
        // a unit.
        if end < bytes.len() && matches!(bytes[end], b'e' | b'E') {
            let mut k = end + 1;
            if k < bytes.len() && matches!(bytes[k], b'+' | b'-') {
                k += 1;
            }
            if k < bytes.len() && bytes[k].is_ascii_digit() {
                while k < bytes.len() && bytes[k].is_ascii_digit() {
                    k += 1;
                }
                end = k;
            }
        }
        let mantissa: f64 = rest[..end]
            .parse()
            .map_err(|_| self.error("malformed number"))?;
        // SPICE's scale suffix, then any unit letters, which are ignored.
        let letters = rest[end..]
            .find(|c: char| !c.is_ascii_alphabetic())
            .unwrap_or(rest.len() - end);
        let suffix = rest[end..end + letters].to_ascii_lowercase();
        let scale = if suffix.starts_with("meg") {
            1e6
        } else if suffix.starts_with("mil") {
            25.4e-6
        } else {
            match suffix.chars().next() {
                Some('t') => 1e12,
                Some('g') => 1e9,
                Some('k') => 1e3,
                Some('m') => 1e-3,
                Some('u') => 1e-6,
                Some('n') => 1e-9,
                Some('p') => 1e-12,
                Some('f') => 1e-15,
                _ => 1.0,
            }
        };
        let value = mantissa * scale;
        if !value.is_finite() {
            return Err(self.error("number out of range"));
        }
        self.pos += end + letters;
        Ok(Node::Num(value))
    }
}

// ---------------------------------------------------------- evaluation

fn truth(x: f64) -> f64 {
    if x != 0.0 {
        1.0
    } else {
        0.0
    }
}

fn apply(op: Op, a: f64, b: f64) -> f64 {
    match op {
        Op::Or => truth(truth(a) + truth(b)),
        Op::And => truth(a) * truth(b),
        Op::Eq => f64::from(u8::from(a == b)),
        Op::Ne => f64::from(u8::from(a != b)),
        Op::Lt => f64::from(u8::from(a < b)),
        Op::Le => f64::from(u8::from(a <= b)),
        Op::Gt => f64::from(u8::from(a > b)),
        Op::Ge => f64::from(u8::from(a >= b)),
        Op::Add => a + b,
        Op::Sub => a - b,
        Op::Mul => a * b,
        Op::Div => a / b,
        Op::Pow => a.powf(b),
    }
}

fn call(func: Func, x: &[f64]) -> f64 {
    match func {
        Func::Abs => x[0].abs(),
        Func::Sqrt => x[0].sqrt(),
        Func::Exp => x[0].exp(),
        Func::Ln | Func::Log => x[0].ln(),
        Func::Log10 => x[0].log10(),
        Func::Sin => x[0].sin(),
        Func::Cos => x[0].cos(),
        Func::Tan => x[0].tan(),
        Func::Asin => x[0].asin(),
        Func::Acos => x[0].acos(),
        Func::Atan => x[0].atan(),
        Func::Sinh => x[0].sinh(),
        Func::Cosh => x[0].cosh(),
        Func::Tanh => x[0].tanh(),
        Func::U => f64::from(u8::from(x[0] > 0.0)),
        Func::Uramp => x[0].max(0.0),
        Func::Sgn => {
            if x[0] > 0.0 {
                1.0
            } else if x[0] < 0.0 {
                -1.0
            } else {
                0.0
            }
        }
        Func::Floor => x[0].floor(),
        Func::Ceil => x[0].ceil(),
        Func::Atan2 => x[0].atan2(x[1]),
        Func::Pow => x[0].powf(x[1]),
        Func::Pwr => x[0].abs().powf(x[1]).copysign(x[0]),
        Func::Min => x[0].min(x[1]),
        Func::Max => x[0].max(x[1]),
        Func::Limit => x[0].max(x[1]).min(x[2]),
    }
}

fn eval(node: &Node, values: &[f64]) -> f64 {
    match node {
        Node::Num(v) => *v,
        Node::Var(k) => values.get(*k).copied().unwrap_or(0.0),
        Node::Neg(a) => -eval(a, values),
        Node::Not(a) => 1.0 - truth(eval(a, values)),
        Node::Bin(op, a, b) => apply(*op, eval(a, values), eval(b, values)),
        Node::Call(func, args) => {
            let x: Vec<f64> = args.iter().map(|a| eval(a, values)).collect();
            call(*func, &x)
        }
        Node::Cond(c, a, b) => {
            if eval(c, values) != 0.0 {
                eval(a, values)
            } else {
                eval(b, values)
            }
        }
    }
}

/// A value and its gradient by each variable.
struct Dual {
    v: f64,
    d: Vec<f64>,
}

impl Dual {
    fn constant(v: f64, n: usize) -> Dual {
        Dual { v, d: vec![0.0; n] }
    }

    /// `f(self)` with `f'(self)` given: the chain rule.
    fn chain(mut self, v: f64, slope: f64) -> Dual {
        self.v = v;
        for d in &mut self.d {
            *d *= slope;
        }
        self
    }

    /// `α·self + β·other` in the gradient, with the value `v`.
    fn combine(mut self, alpha: f64, other: &Dual, beta: f64, v: f64) -> Dual {
        self.v = v;
        for (d, o) in self.d.iter_mut().zip(&other.d) {
            *d = alpha * *d + beta * o;
        }
        self
    }
}

fn eval_dual(node: &Node, values: &[f64], n: usize) -> Dual {
    match node {
        Node::Num(v) => Dual::constant(*v, n),
        Node::Var(k) => {
            let mut d = Dual::constant(values.get(*k).copied().unwrap_or(0.0), n);
            d.d[*k] = 1.0;
            d
        }
        Node::Neg(a) => {
            let a = eval_dual(a, values, n);
            let v = -a.v;
            a.chain(v, -1.0)
        }
        Node::Not(a) => Dual::constant(1.0 - truth(eval_dual(a, values, n).v), n),
        Node::Bin(op, a, b) => {
            let a = eval_dual(a, values, n);
            let b = eval_dual(b, values, n);
            let v = apply(*op, a.v, b.v);
            match op {
                Op::Add => a.combine(1.0, &b, 1.0, v),
                Op::Sub => a.combine(1.0, &b, -1.0, v),
                Op::Mul => {
                    let (av, bv) = (a.v, b.v);
                    a.combine(bv, &b, av, v)
                }
                Op::Div => {
                    let (av, bv) = (a.v, b.v);
                    a.combine(1.0 / bv, &b, -av / (bv * bv), v)
                }
                Op::Pow => {
                    let (av, bv) = (a.v, b.v);
                    // d(a^b) = b·a^(b−1)·da + a^b·ln(a)·db; the second
                    // term only where b varies, so a negative base with a
                    // constant exponent stays finite.
                    let da = if bv == 0.0 {
                        0.0
                    } else {
                        bv * av.powf(bv - 1.0)
                    };
                    let db = if b.d.iter().all(|d| *d == 0.0) {
                        0.0
                    } else {
                        v * av.ln()
                    };
                    a.combine(da, &b, db, v)
                }
                _ => Dual::constant(v, n),
            }
        }
        Node::Call(func, args) => {
            let x: Vec<Dual> = args.iter().map(|a| eval_dual(a, values, n)).collect();
            let xv: Vec<f64> = x.iter().map(|d| d.v).collect();
            let v = call(*func, &xv);
            let mut x = x.into_iter();
            let a = x.next().expect("every function takes an argument");
            let x0 = xv[0];
            let slope = match func {
                Func::Abs => {
                    if x0 >= 0.0 {
                        1.0
                    } else {
                        -1.0
                    }
                }
                Func::Sqrt => 0.5 / v,
                Func::Exp => v,
                Func::Ln | Func::Log => 1.0 / x0,
                Func::Log10 => 1.0 / (x0 * std::f64::consts::LN_10),
                Func::Sin => x0.cos(),
                Func::Cos => -x0.sin(),
                Func::Tan => 1.0 + v * v,
                Func::Asin => 1.0 / (1.0 - x0 * x0).sqrt(),
                Func::Acos => -1.0 / (1.0 - x0 * x0).sqrt(),
                Func::Atan => 1.0 / (1.0 + x0 * x0),
                Func::Sinh => x0.cosh(),
                Func::Cosh => x0.sinh(),
                Func::Tanh => 1.0 - v * v,
                Func::Uramp => {
                    if x0 > 0.0 {
                        1.0
                    } else {
                        0.0
                    }
                }
                Func::U | Func::Sgn | Func::Floor | Func::Ceil => 0.0,
                Func::Atan2 => {
                    let b = x.next().expect("two arguments");
                    let (y, xx) = (x0, xv[1]);
                    let r2 = y * y + xx * xx;
                    return a.combine(xx / r2, &b, -y / r2, v);
                }
                Func::Pow | Func::Pwr => {
                    let b = x.next().expect("two arguments");
                    let (base, e) = (x0, xv[1]);
                    // pow is a^e; pwr is sgn(a)·|a|^e, whose slope is
                    // e·|a|^(e−1) on both sides.
                    let da = match (e == 0.0, *func) {
                        (true, _) => 0.0,
                        (false, Func::Pow) => e * base.powf(e - 1.0),
                        (false, _) => e * base.abs().powf(e - 1.0),
                    };
                    let db = if b.d.iter().all(|d| *d == 0.0) {
                        0.0
                    } else {
                        v * base.abs().ln()
                    };
                    return a.combine(da, &b, db, v);
                }
                Func::Min | Func::Max => {
                    let b = x.next().expect("two arguments");
                    let first = (*func == Func::Min) == (x0 <= xv[1]);
                    return if first {
                        a.chain(v, 1.0)
                    } else {
                        b.chain(v, 1.0)
                    };
                }
                Func::Limit => {
                    if x0 < xv[1] {
                        return x.next().expect("three arguments").chain(v, 1.0);
                    }
                    if x0 > xv[2] {
                        return x.nth(1).expect("three arguments").chain(v, 1.0);
                    }
                    1.0
                }
            };
            a.chain(v, slope)
        }
        Node::Cond(c, a, b) => {
            if eval(c, values) != 0.0 {
                eval_dual(a, values, n)
            } else {
                eval_dual(b, values, n)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ops::Add;

    fn e(text: &str) -> Expression {
        Expression::parse(text).unwrap_or_else(|err| panic!("{err}"))
    }

    #[test]
    fn precedence_and_associativity() {
        assert_eq!(e("1+2*3").evaluate(&[]), 7.0);
        assert_eq!(e("2^3^2").evaluate(&[]), 512.0);
        assert_eq!(e("-2^2").evaluate(&[]), -4.0);
        assert_eq!(e("2^-1").evaluate(&[]), 0.5);
        assert_eq!(e("2**3").evaluate(&[]), 8.0);
        assert_eq!(e("10-4-3").evaluate(&[]), 3.0);
        assert_eq!(e("8/4/2").evaluate(&[]), 1.0);
        assert_eq!(e("1 < 2 && 3 > 4 || !0").evaluate(&[]), 1.0);
        assert_eq!(e("0 ? 1 : 2 ? 3 : 4").evaluate(&[]), 3.0);
    }

    #[test]
    fn numbers_take_spice_suffixes_and_ignore_units() {
        for (text, want) in [
            ("4.7k", 4.7e3),
            ("10meg", 10e6),
            ("26m", 26e-3),
            ("1u", 1e-6),
            ("3n", 3e-9),
            ("2p", 2e-12),
            ("5f", 5e-15),
            ("1t", 1e12),
            ("2g", 2e9),
            ("1mil", 25.4e-6),
            ("1e3", 1e3),
            ("1.5e-3", 1.5e-3),
            ("1kohm", 1e3),
            ("1v", 1.0),
            (".5", 0.5),
        ] {
            let got = e(text).evaluate(&[]);
            assert!((got - want).abs() <= 1e-12 * want.abs(), "{text}: {got}");
        }
    }

    #[test]
    fn variables_are_shared_and_ordered() {
        let x = e("v(a)*v(a) + v(b, a) + i(V1) + TIME + V(A)");
        assert_eq!(
            x.variables(),
            [
                Variable::Voltage {
                    node: "a".into(),
                    reference: None
                },
                Variable::Voltage {
                    node: "b".into(),
                    reference: Some("a".into())
                },
                Variable::Current {
                    source: "V1".into()
                },
                Variable::Time,
                Variable::Voltage {
                    node: "A".into(),
                    reference: None
                },
            ]
        );
    }

    #[test]
    fn display_round_trips() {
        for text in [
            "v(a)*v(b)+0.002",
            "-(v(a)-v(b))^2",
            "(1+2)*3",
            "1-(2-3)",
            "2^3^2",
            "(2^3)^2",
            "v(in,ref)>0?tanh(v(x)/0.026):-1",
            "limit(i(Vs)*1000,-1,1)",
            "!(time<1e-6)",
            "(-2)^2",
            "(v(a)<1)==1",
        ] {
            let x = e(text);
            let back = e(&x.to_string());
            assert_eq!(back, x, "{text} → {x}");
        }
        assert_eq!(e("1 - (2 - 3)").to_string(), "1-(2-3)");
        assert_eq!(e("(1 - 2) - 3").to_string(), "1-2-3");
        assert_eq!(e("1e-12*v(a)").to_string(), "1e-12*v(a)");
    }

    #[test]
    fn errors_say_where() {
        for (text, says) in [
            ("v(a", "expected `)`"),
            ("foo(1)", "unknown function `foo`"),
            ("x + 1", "unknown name `x`"),
            ("min(1)", "`min` takes 2 arguments, not 1"),
            ("1 +", "expected a value"),
            ("1 2", "unexpected text"),
            ("1 < 2 < 3", "comparisons don't chain"),
            ("v()", "expected a node or source name"),
            (&"(".repeat(10_000), "nested too deeply"),
            (&"-".repeat(10_000), "nested too deeply"),
            (&"1+".repeat(10_000).add("1"), "too many operators"),
            ("1e400", "number out of range"),
            ("", "expected a value"),
        ] {
            let err = Expression::parse(text).unwrap_err();
            assert!(err.message.contains(says), "{text}: {err}");
        }
        assert_eq!(Expression::parse("x + 1").unwrap_err().position, 0);
    }

    #[test]
    fn constants_resolve_through_the_lookup() {
        let x =
            Expression::parse_with("gain*v(a)", &|name| (name == "gain").then_some(4.0)).unwrap();
        assert_eq!(x.evaluate(&[2.0]), 8.0);
        // A negative constant prints in parentheses and reads back.
        let x = Expression::parse_with("k^2", &|_| Some(-3.0)).unwrap();
        assert_eq!(x.to_string(), "(-3)^2");
        assert_eq!(x.evaluate(&[]), 9.0);
        let err = Expression::parse_with("k", &|_| Some(f64::INFINITY)).unwrap_err();
        assert!(err.message.contains("`k` is inf"), "{err}");
    }

    #[test]
    fn from_str_and_missing_values() {
        let x: Expression = "v(a)+1".parse().unwrap();
        // A value not given reads as 0.
        assert_eq!(x.evaluate(&[]), 1.0);
        assert!("".parse::<Expression>().is_err());
    }

    #[test]
    fn every_function_evaluates() {
        for (text, want) in [
            ("abs(-2)", 2.0),
            ("sqrt(9)", 3.0),
            ("exp(0)", 1.0),
            ("ln(1)", 0.0),
            ("log(1)", 0.0),
            ("log10(100)", 2.0),
            ("sin(0)", 0.0),
            ("cos(0)", 1.0),
            ("tan(0)", 0.0),
            ("asin(0)", 0.0),
            ("acos(1)", 0.0),
            ("atan(0)", 0.0),
            ("sinh(0)", 0.0),
            ("cosh(0)", 1.0),
            ("tanh(0)", 0.0),
            ("u(1)+u(0)", 1.0),
            ("uramp(-1)+uramp(2)", 2.0),
            ("sgn(-3)+sgn(0)+sgn(5)", 0.0),
            ("floor(1.5)+ceil(1.5)", 3.0),
            ("atan2(0,1)", 0.0),
            ("pow(2,3)", 8.0),
            ("pwr(-2,2)", -4.0),
            ("min(1,2)+max(1,2)", 3.0),
            ("limit(5,0,1)+limit(-5,0,1)+limit(0.5,0,1)", 1.5),
            ("1==1", 1.0),
            ("1!=1", 0.0),
            ("1<=1", 1.0),
            ("2>=3", 0.0),
            ("0||0", 0.0),
            ("1&&0", 0.0),
            ("!1", 0.0),
            ("+3", 3.0),
            ("-pi/pi", -1.0),
        ] {
            let got = e(text).evaluate(&[]);
            assert!((got - want).abs() < 1e-12, "{text}: {got}");
        }
    }

    /// Every function's gradient against a central difference.
    #[test]
    fn gradients_match_finite_differences() {
        let cases = [
            "v(a)*v(b)",
            "v(a)/v(b)",
            "v(a)^v(b)",
            "v(a)^3",
            "pow(v(a),2.5)",
            "pwr(v(a)-2,1.5)",
            "abs(v(a)-2)",
            "sqrt(v(a))",
            "exp(v(a)/2)",
            "ln(v(a))",
            "log(v(a))",
            "log10(v(a))",
            "sin(v(a))",
            "cos(v(a))",
            "tan(v(a)/4)",
            "asin(v(a)/4)",
            "acos(v(a)/4)",
            "atan(v(a))",
            "sinh(v(a))",
            "cosh(v(a))",
            "tanh(v(a)-1)",
            "uramp(v(a)-1)",
            "atan2(v(a),v(b))",
            "min(v(a),v(b))",
            "max(v(a),v(b))",
            "limit(v(a),0,v(b))",
            "limit(v(b),0,1)",
            "limit(-v(a),0,1)",
            "v(a)>1?v(a)*v(b):-v(b)",
            "-v(a)+i(V1)*time",
            "u(v(a))+sgn(v(a))+floor(v(a))+ceil(v(a))",
        ];
        let at = [1.3, 2.1, 0.7, 0.4];
        for text in cases {
            let x = e(text);
            let n = x.variables().len();
            let values = &at[..n];
            let mut gradient = vec![0.0; n];
            let value = x.evaluate_with_gradient(values, &mut gradient);
            assert!((value - x.evaluate(values)).abs() < 1e-12, "{text}");
            for k in 0..n {
                let h = 1e-6;
                let mut up = values.to_vec();
                let mut down = values.to_vec();
                up[k] += h;
                down[k] -= h;
                let fd = (x.evaluate(&up) - x.evaluate(&down)) / (2.0 * h);
                assert!(
                    (gradient[k] - fd).abs() <= 1e-6 * fd.abs().max(1.0),
                    "{text} ∂/∂{k}: {} vs {fd}",
                    gradient[k]
                );
            }
        }
    }

    #[test]
    fn negative_base_with_a_constant_exponent_has_a_finite_gradient() {
        let x = e("v(a)^3");
        let mut g = [0.0];
        assert_eq!(x.evaluate_with_gradient(&[-2.0], &mut g), -8.0);
        assert_eq!(g[0], 12.0);
        let x = e("pow(v(a),2)");
        assert_eq!(x.evaluate_with_gradient(&[-3.0], &mut g), 9.0);
        assert_eq!(g[0], -6.0);
    }

    #[test]
    fn renaming_follows_a_hierarchy() {
        let mut x = e("v(a,b)*i(V1)");
        x.rename(|n| format!("x1.{n}"), |s| format!("x1.{s}"));
        assert_eq!(x.to_string(), "v(x1.a,x1.b)*i(x1.V1)");
        assert_eq!(x.nodes().collect::<Vec<_>>(), ["x1.a", "x1.b"]);
    }

    #[cfg(feature = "serde")]
    #[test]
    fn serde_is_the_text() {
        let x = e("2*v(a)");
        let json = serde_json::to_string(&x).unwrap();
        assert_eq!(json, "\"2*v(a)\"");
        assert_eq!(serde_json::from_str::<Expression>(&json).unwrap(), x);
        assert!(serde_json::from_str::<Expression>("\"2*\"").is_err());
    }
}
