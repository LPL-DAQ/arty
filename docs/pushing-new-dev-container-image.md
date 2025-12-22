# Updating the dev container image

## Setup credentials

Create a new classic personal access token [here](https://github.com/settings/tokens/new). Ensure that you add
permissions for "write:packages".

Ensure that `~/arty/credentials` exists.

Add two variables into the file specifying your Github username and the personal access token you just created, as so:

```text
# In ~/arty/credentials:
GITHUB_USERNAME=jamm-es
GITHUB_PERSONAL_ACCESS_TOKEN=ghp_[...]
```

## Build and push

All devcontainer images are build from the root Dockerfile. Modify it to include your change and commit all your
changes to a new branch.

Then, run:

```shell
scripts/build-dev-container-image.sh
```

This will re-build the image, push it to the Github Container Registry, update the dev container config so
your image is used, and push all changes.

## Managing images

All images being served under the `lpl-daq` org can be viewed and cleaned up [at this page](https://github.com/orgs/lpl-daq/packages).
