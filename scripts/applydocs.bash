#!/bin/bash
branch_name=`git rev-parse --abbrev-ref HEAD`
if ( ! git diff-index --quiet HEAD -- ); then
  echo "ERROR: uncommited changes exist"
  exit 1
else
  # Go to root of git
  cd `git rev-parse --show-toplevel`
  echo "Generating docs"
  doxygen > /dev/null
  git config merge.renameLimit 999999
  echo "Adding docs to git"
  git add -f docs > /dev/null
  echo "Stashing docs"
  git stash  > /dev/null
  echo "Checking gh-pages"
  git checkout gh-pages > /dev/null
  echo "Pull remote changes"
  git pull
  echo "Applying docs to gh-pages"
  git stash pop > /dev/null 2> /dev/null
  set -o pipefail
  grep --exclude=\*.{bash,swp} -lr '<<<<<<< ' . | xargs git checkout --theirs
  error=$?
  if [ $error -ne 0 ]; then
    echo "Failed to checkout applydocs"
    echo "Reverting to original state"
    git reset HEAD
    git checkout .
    git checkout $branch_name
    exit 1
  fi
  git add -u . 
  git config --unset merge.renameLimit
  if ( ! git diff-index --quiet HEAD -- ); then
    echo "Commiting docs"
    git commit -m "Applying Docs from $branch_name on $( date )"
    echo "Pushing docs"
    git push --no-verify
    error=$?
    if [ $error -ne 0 ]; then
      echo "Failed to push; Set git branch --set-upstream-to=[GH_PAGES_BRANCH_REMOTE]"
      exit 1
    fi
  else
    echo "Nothing to commit"
  fi
  echo "Reverting to original branch"
  git checkout $branch_name
fi
