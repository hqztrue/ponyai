# Copyright 2017 The TensorFlow Authors. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Utilities for defining Pony Bazel dependencies."""

def _is_windows(ctx):
  return ctx.os.name.lower().find("windows") != -1

def _get_env_var(ctx, name):
  if name in ctx.os.environ:
    return ctx.os.environ[name]
  else:
    return None

# Executes specified command with arguments and calls 'fail' if it exited with
# non-zero code
def _execute_and_check_ret_code(repo_ctx, cmd_and_args):
  result = repo_ctx.execute(cmd_and_args, timeout=10)
  if result.return_code != 0:
    fail(("Non-zero return code({1}) when executing '{0}':\n" + "Stdout: {2}\n"
          + "Stderr: {3}").format(" ".join(cmd_and_args), result.return_code,
                                  result.stdout, result.stderr))

def _to_label(path):
  return Label("//" + path[::-1].replace("/", ":", 1)[::-1])

# Apply a patch_file to the repository root directory
# Runs 'patch -p1'
def _apply_patch(ctx, patch_file):
  # Don't check patch on Windows, because patch is only available under bash.
  if not _is_windows(ctx) and not ctx.which("patch"):
    fail("patch command is not found, please install it")
  cmd = ["patch", "-p1", "-d", ctx.path("."), "-i", ctx.path(patch_file)]
  if _is_windows(ctx):
    bazel_sh = _get_env_var(ctx, "BAZEL_SH")
    if not bazel_sh:
      fail("BAZEL_SH environment variable is not set")
    cmd = [bazel_sh, "-c", " ".join(cmd)]
  _execute_and_check_ret_code(ctx, cmd)

def _apply_delete(ctx, paths):
  for path in paths:
    if path.startswith("/"):
      fail("refusing to rm -rf path starting with '/': " + path)
    if ".." in path:
      fail("refusing to rm -rf path containing '..': " + path)
  _execute_and_check_ret_code(
      ctx, ["rm", "-rf"] + [ctx.path(path) for path in paths])

def _pony_http_archive(ctx):
  ctx.download_and_extract(
      ctx.attr.urls,
      "",
      ctx.attr.sha256,
      ctx.attr.type,
      ctx.attr.strip_prefix)
  if ctx.attr.delete:
    _apply_delete(ctx, ctx.attr.delete)
  if ctx.attr.patch_file != "":
    _apply_patch(ctx, _to_label(ctx.attr.patch_file))
  if ctx.attr.build_file != "":
    ctx.symlink(_to_label(ctx.attr.build_file), "BUILD.bazel")

pony_http_archive = repository_rule(
    implementation=_pony_http_archive,
    attrs={
        "sha256": attr.string(mandatory=True),
        "urls": attr.string_list(mandatory=True, allow_empty=False),
        "strip_prefix": attr.string(),
        "type": attr.string(),
        "delete": attr.string_list(),
        "patch_file": attr.string(),
        "build_file": attr.string(),
    })
"""Downloads and creates Bazel repos for dependencies.
This is a swappable replacement for both http_archive() and
new_http_archive() that offers some additional features. It also helps
ensure best practices are followed.
"""
