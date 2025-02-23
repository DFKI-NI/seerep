# Available Modules

## common.py

Contains common functions like `get_grpc_channel()` and some functions for
passing into `boltons.iterutils.remap`.

The source is located
[here](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/util/common.py).

## fb_helper.py

This is a module provided through the
[seerep_grpc](https://pypi.org/project/seerep-grpc/) package. It provides
functions to construct a subset of the SEEREP flatbuffers message in one
function call. Most of the functions only return the component for further
composition into another datatype. If the component itself should be serialized
`builder.Finish()` as well as `builder.Output()` need to be called, on the from
the function retrieved value.

The source is located
[here](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/util/fb_helper.py).

## service_manager.py

Contains a class `ServiceManager` which when instantiated can be used to
instantly call the services with the return values of the `fb_helper.py`
functions, i.e. the `builder.Finish()` and `builder.Output()` calls are done in
the `ServiceManager` methods. Example usage can be found in
[msg_abs/msgs.py](https://github.com/DFKI-NI/seerep/blob/main/tests/python/gRPC/util/msg_abs/msgs.py)
, e.g.:

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/tests/python/gRPC/util/msg_abs/msgs.py:274:309"
```

The source is located
[here](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/util/service_manager.py).

**Note**: This module is currently incomplete and contains only a subset of the
available service calls.

## fb_to_dict.py

### fb_flatc_dict()

The `fb_flatc_dict` function can be used to convert serialized `flatbuffers`
objects to a python dictionary. More information can be found by looking at the
docstring:

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/examples/python/gRPC/util/fb_to_dict.py:105:119"
```

### catkin_find_schema_dir()

Looking at the docstring should reveal everything to know about this function:

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/examples/python/gRPC/util/fb_to_dict.py:63:77"
```

### SchemaFileNames

A enum type class, to map the available flatbuffers type schema file names.
Only used as a type specifier for `fb_flatc_dict`.
