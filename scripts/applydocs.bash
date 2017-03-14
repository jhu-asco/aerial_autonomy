#!/bin/bash
branch_name=`git rev-parse --abbrev-ref HEAD`
if ( ! git diff-index --quiet HEAD -- ); then
  echo "ERROR: uncommited changes exist"
  exit 1
else
  doxygen
  git config merge.renameLimit 999999
  echo "Adding docs to git"
  git add -f docs > /dev/null
  echo "Stashing docs"
  git stash  > /dev/null
  echo "Checking gh-pages"
  git checkout gh-pages > /dev/null
  echo "Applying docs to gh-pages"
  git stash pop > /dev/null 2> /dev/null
  grep -lr '<<<<<<< ' . | xargs git checkout --theirs
  git add -u . 
  git config --unset merge.renameLimit
  echo "Commiting docs"
  git commit -m "Applying Docs from $branch_name on $( date )"
  echo "Reverting to original branch"
  git checkout $branch_name
fi
