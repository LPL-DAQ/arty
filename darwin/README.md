# darwin

> I hate myself, I hate clover, and I hate bees.
>
> — Charles Darwin, September 3rd, 1862

Shared code for clover and bees.

## Build

This library is not built directly; the main clover and bees projects simply
`add_subdirectory` this folder. Thus, we just specify include paths and sources
for the generic `app` target that both projects build for.

When modifying code here, care must be taken to ensure that both clover and bees
still build and function correctly.


