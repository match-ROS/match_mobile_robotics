---
title: My page
layout: default
---

Update dependencies
```
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Clone a repository without submodules
```
git clone <repo_url>
```

Clone a repository with submodules
```
git clone <repo_url>
cd <repo_name>
git submodule init
git submodule update
```