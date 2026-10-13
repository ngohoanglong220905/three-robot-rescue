# Standing person asset

- Author: Marina Kollmitz; created with MakeHuman and Blender.
- Source: https://github.com/osrf/gazebo_models/tree/8163eb4b5e7e21985c6591d1c0bfb56468c0093f/person_standing
- Upstream repository license: Creative Commons Attribution 3.0 Unported; see `LICENSE`.
- License terms: https://creativecommons.org/licenses/by/3.0/
- Mesh, textures, `model.sdf` and `model.config` were copied without modification.
- `meshes/seated.dae` is a derived static seated pose, generated locally by
  `scripts/generate_seated_person.py`: hips/knees bent, torso lowered, normals
  transformed; original UVs and textures retained. It shares the same CC BY 3.0
  attribution to Marina Kollmitz. The original `standing.dae` is unchanged.
- `social_rescue.sdf` reuses the mesh at scale 0.92 with a rotation to align its
  face with local +X, and supplies a static cylindrical collision envelope.
  The scene does not use the upstream physics model or claim action animations.
