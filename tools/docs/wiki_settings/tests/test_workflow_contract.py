from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[4]
CHECK_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "wiki-settings-check.yaml"
PUBLISH_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "wiki-settings-publish.yaml"


class WikiSettingsWorkflowContractTest(unittest.TestCase):
  def test_pull_request_workflow_is_read_only(self):
    text = CHECK_WORKFLOW.read_text(encoding="utf-8")
    self.assertIn("pull_request:", text)
    self.assertIn("contents: read", text)
    self.assertNotIn("contents: write", text)
    self.assertIn("persist-credentials: false", text)
    self.assertNotIn("secrets.", text)

  def test_publish_workflow_updates_only_the_actual_wiki(self):
    text = PUBLISH_WORKFLOW.read_text(encoding="utf-8")
    self.assertTrue(text.startswith("name: cp-set-wiki\n"))
    self.assertIn("- carrot-wip", text)
    self.assertIn("gollum:", text)
    self.assertIn("workflow_dispatch:", text)
    self.assertNotIn("pull_request:", text)
    self.assertIn("contents: write", text)
    self.assertIn("cancel-in-progress: false", text)
    self.assertIn("${{ github.repository }}.wiki.git", text)
    self.assertIn("git -C \"$RUNNER_TEMP/carrot-wiki\" push \"$WIKI_PUSH_URL\" HEAD", text)
    self.assertNotIn("wiki-settings-preview", text)
    self.assertNotIn("wiki-settings-published", text)
    self.assertNotIn("git worktree", text)
    self.assertNotIn("checkout --orphan", text)


if __name__ == "__main__":
  unittest.main()
