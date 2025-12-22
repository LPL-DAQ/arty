# floyd

**F**orwarding **L**ots **O**f **Y**our **D**ata.

## Build

Make sure to run from the dev container.

```shell
cmake -S ~/clover/floyd -B ~/clover/floyd/build -G Ninja
ninja -C floyd/build install
```

If you have any weird build issues, try removing the build directory then re-building, like so:

```shell
rm -rf ~/clover/floyd/build
```

## Run

Make sure to run from `~/clover` in the dev container.

```shell
floyd
```
