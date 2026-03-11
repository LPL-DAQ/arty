# Updating dev container version

When the base dev container image is updated, it is necessary to rebuild the container
to use the new image. To do so, use the following steps:

1. Ensure all changes are pushed to GitHub. We are about to erase all local content,
so any unpushed changes will be lost.

2. Close out of the dev container environment.

3. Open Docker Desktop and stop the dev container. Then, delete it.

4. Open the volumes tab on the left side, then delete the dev container volume as well.

5. Open Visual Studio Code, then re-build the container by:

    1. Opening the command palette with `Ctrl-Shift-P` or `Cmd-Shift-P`.

    2. Search for and select "Dev Containers: Clone Repository in Container Volume".

    3. Select "Clone a repository from Github..." and authenticate if necessary.

    4. Select the `arty` repo.

    5. Select the main branch.

6. Once the dev container is done building and re-opens, you may then check out the
remote branch you were working on.

7. To ensure that the old container image isn't taking up any disk space, open the
terminal and run:

```shell
docker image prune --all
```
