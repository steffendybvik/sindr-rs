use sindr::{examples, solve_circuit};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let circuit = examples::npn_common_emitter();
    let result = solve_circuit(&circuit)?;

    let bjt = result
        .bjt_results
        .first()
        .ok_or("expected at least one BJT in the result")?;

    println!("=== NPN Common Emitter Operating Point ===");
    println!("Vbe    = {:.4} V", bjt.vbe);
    println!("Vce    = {:.4} V", bjt.vce);
    println!("Ib     = {:.4} mA", bjt.ib * 1e3);
    println!("Ic     = {:.4} mA", bjt.ic * 1e3);
    println!("Ie     = {:.4} mA", bjt.ie * 1e3);
    println!("Region : {}", bjt.region);
    println!("Power  = {:.4} mW", bjt.power * 1e3);

    Ok(())
}
