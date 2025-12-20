#pragma once
#include "pxr\base\tf\hash.h"
#include "pxr\base\tf\hashmap.h"

USTC_CG_NAMESPACE_OPEN_SCOPE
    class Hd_USTC_CG_Light;
    class Hd_USTC_CG_Camera;
    class Hd_USTC_CG_Mesh;
    class Hd_USTC_CG_Material;

    using LightArray = VtArray<Hd_USTC_CG_Light*>;
    using CameraArray = VtArray<Hd_USTC_CG_Camera*>;
    using MeshArray = VtArray<Hd_USTC_CG_Mesh*>;
    using MaterialMap = TfHashMap<SdfPath, Hd_USTC_CG_Material*, TfHash>;
USTC_CG_NAMESPACE_CLOSE_SCOPE
