# Writing integration tests for example pyscripts

Before a test is written it makes sense to understand how a example is structured.
For this see [writing-examples.md](writing-python-examples.md) as a introduction.
How tests can be structured will be demonstrated on the already created
`test_ gRPC_fb_addLabels` test found
[here](https://github.com/DFKI-NI/seerep/blob/main/tests/python/gRPC/images/test_gRPC_fb_addLabels.py).

As the testing framework for python [pytest](https://github.com/pytest-dev/pytest/)
is used.

## The code

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/tests/python/gRPC/images/test_gRPC_fb_addLabels.py:10:18"
```

For the imported modules, note that the examples themselves are imported to be
used in the tests. Furthermore helper functions from `seerep.util.fb_helper`,
as well as `fb_flatc_dict` are imported.

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/tests/python/gRPC/images/test_gRPC_fb_addLabels.py:21:33"
```

This is a helper function to retrieve all images the targeted project on the
targeted server has. At the end with the service call `GetImage()` the images
are returned in a list of bytearray objects, which on the return line are
converted to a python dictionary with the help of the `SchemaFileNames` enum.
`SchemaFileNames` contains references to the file names of the datatypes of all
the SEEREP flatbuffers types, this is required for the `flatc` compiler to know
how to decode the object.

```python
--8<-- "https://raw.githubusercontent.com/DFKI-NI/seerep/main/tests/python/gRPC/images/test_gRPC_fb_addLabels.py:36:75"
```

Next up the test function is defined and uses the fixture `grpc_channel`, which
spins the test server up, creates a channel to that server and makes sure that
the server is terminated after testing is done. The fixture `project_setup`
creates a test project on the server and deletes that after the testing function
using SEEREP server calls. Implementations of both fixtures can be found
[here](https://github.com/DFKI-NI/seerep/blob/main/tests/conftest.py).

After that the `project_uuid` of the created project is retrieved and used for
the different calls to the example functions. To attach new Labels to
images, it has to be ensured that images are present. This is done by utilizing
the [gRPC_pb_sendLabeledImage](https://github.com/DFKI-NI/seerep/blob/main/examples/python/gRPC/images/gRPC_pb_sendLabeledImage.py)
example. Then the Labels example can be used to add Labels to
those images.

Following on the Labels returned by `add_label_raw()` are tested and
compared against those now persisting on the server. `get_imgs` retrieves all
the images from the server, each mapping of image `uuid` to Labels is
iterated and in the list of all images the image, which matches the image `uuid`
of the mapping, is inspected further. Only the Labels with the category
`laterAddedBB` are relevant and therefore filtered. Lastly the sent Labels
are converted to python dictionaries and compared for matching with the filtered
image attached Labels.

## Tips to ease development

- `fb_flatc_dict()` in conjunction with `pytest -s` command is a good option for
debugging tests, as the dictionary and therefore the objects data contents can
be printed that way.
- If a recursive operation has to be applied to a dictionary `boltons`
[remap()](https://boltons.readthedocs.io/en/latest/iterutils.html#boltons.iterutils.remap)
function can be used, like in
[test_gRPC_fb_createGeodeticCoordProject.py](https://github.com/DFKI-NI/seerep/blob/main/tests/python/gRPC/meta/test_gRPC_fb_createGeodeticCoordProject.py).
- If a test should contain a lot of variations in the components of a datatype a
look [here](../reference/pytests-message-abstractions.md) could simplify things
