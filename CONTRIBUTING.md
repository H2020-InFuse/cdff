## Contributing to the CDFF

### Workflow

We use the popular Git **feature branch workflow** with **merge requests**.

In short, developpers develop new features and bug fixes in **feature branches** named anything but `master` in their **local repositories** cloned from the one on the `origin` **remote**, which is Space Apps' server. When they feel that a branch is ready to be shared with the other developers, they can **push** it to the `origin` remote (they mustn't push their branch before it is at least somewhat ready to be shared). When they feel that the branch is ready to be integrated into the main development history, they must request that it is merged into the `master` branch of the `origin` remote.

The **merge request** is easiest initiated through GitLab's web interface. The developer shall write a detailed description of the changes introduced by their branch, and assign the merge request to the most adequate reviewer. Everyone though, not just the reviewer, is encouraged to review and comment on the merge request.

* Once enough positive feedback is obtained, and all objections have been addressed, the branch can be **merged** into `master` on `origin`. The original developer must of course never merge their branch themselves (accept their own merge request) before they have left enough time for their request to be reviewed and gather enough positive feedback.
* If the merge request is rejected, or if the original developer cancels it, the request must be **closed** without the branch being merged. This is the case, for instance, of an idea that needs substantial further work.

In the case of an accepted merge request, the original developper, and everyone else, can finally **pull** (fetch and merge) the updated `master` into their own `master` branch in their repositories. Last but not least, the original developper must also **delete the merged branch** not just from their own local repository but also from the `origin` remote, and more generally from every remote where they have pushed it.

In the case of a rejected or canceled merge request, the original developper must also **delete the unmerged branch** from the `origin` remote, however they might want to keep it in their local repository if they intend to rework it. Note that one must also **delete their abandonned branches** (abandonned ideas) from the `origin` remote if they pushed them there.

Git will never delete a branch by itself, hence why we have to do it ourselves, otherwise they'll linger around. In particular, Git never automatically deletes a branch after merging it into another, contrary to what the term "merge" can imply.

### Documentation about contributing to the CDFF

* [Dependencies of the CDFF (extended version)](https://drive.google.com/open?id=1Lv1ryzOCpTKXPyYZ77M07PNrthuIvzkSY2At1ib6FX8)
* [Description of the repository filesystem hierarchy](https://drive.google.com/open?id=1ppECSp_fz4f23C0t9v5XJkrxQOHpKud0CJxJu4E5LpI)
* [Help with Git and its jargon](https://drive.google.com/open?id=1b9SNJDLAeYy8wc-1ryeyGpYzZKl0vcmHSz0IqaAfmLI)
* [Coding standard for the CDFF](https://drive.google.com/open?id=1jQ8I3lRKLel6BT5Fac5twtjzZ0SiQrc9rK23v-3NOLM)
* [Tutorial on writing DFNs](https://drive.google.com/open?id=1hFTRKgJNN3n_brT3aajMA03AR_jQ2eCo-ZM33ggY5cE)
* [Tutorial on writing DFPCs](https://drive.google.com/open?id=1ZUhZPnedd1mO42y-q4N7USltOnKeZzbyyZz_yzpLsmk)
