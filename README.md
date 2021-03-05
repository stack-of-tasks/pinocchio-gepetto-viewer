# pinocchio-gepetto-viewer

C++ client to gepetto-viewer for Pinocchio.

It depends on:
- pinocchio
- gepetto-viewer-corba

## Compilation

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install_prefix ..
make install
```

## Usage

### C++ viewer
See *tests/ur5.cc* for an up-to-date example.

```cpp
#include <pinocchio/gepetto/viewer.hpp>

pinocchio::gepetto::Viewer viewer (model,
 &visualModel, // or NULL
 &collisionModel // or NULL
);
viewer.initViewer("pinocchio"); // window name
viewer.loadViewerModel("ur5"); // scene name

viewer.display(pin::neutral(model));
```

### gepetto-viewer plugin

You can launch the example with
```
gepetto-gui --load-plugin pinocchio-plugin.so --run-pyscript tests/pinocchio-plugin.py
```
