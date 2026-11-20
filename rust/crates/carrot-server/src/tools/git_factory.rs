use super::{
    context::{Context, Reply},
    runner::Failure,
};
use crate::Value;

pub(super) async fn fetch(context: &Context) -> Result<Reply, Failure> {
    context.progress("fetch branches from origin", 1, 2)?;
    let configured = context
        .git(&["remote", "set-branches", "origin", "*"], 15, false)
        .await?;
    if configured.code != 0 {
        return context.result(configured, None);
    }
    let fetched = context.native_fetch().await?;
    if fetched.code != 0 {
        return context.result(fetched, None);
    }
    context.progress("list origin branches", 2, 2)?;
    let listed = context
        .git(
            &[
                "for-each-ref",
                "--format=%(refname:strip=3)",
                "refs/remotes/origin/",
            ],
            15,
            false,
        )
        .await?;
    if listed.code != 0 {
        return context.result(listed, None);
    }
    let branches: std::collections::BTreeSet<_> = listed
        .output
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && *line != "HEAD")
        .collect();
    let output = fetched.output;
    context.append(&output)?;
    Ok(Reply::ok(Value::object([
        ("ok", Value::Bool(true)),
        (
            "summary_vars",
            Value::object([("count", Value::integer(branches.len()))]),
        ),
        (
            "branches",
            Value::Array(branches.iter().map(|branch| Value::text(branch)).collect()),
        ),
        ("out", Value::text(&output)),
        (
            "summary_key",
            Value::text("git_result_reset_repo_fetch_done"),
        ),
    ])))
}
