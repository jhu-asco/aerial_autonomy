#!/bin/bash
echo Setting up Git hooks...

git_root=$(git rev-parse --show-toplevel)
git_hooks=$git_root"/setup/hooks/*"
for h in $git_hooks
do
  ln -sf $h $git_root"/.git/hooks/"$(basename $h)
done

echo Done
