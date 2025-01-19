//-----------------------------------------------------------------------------
// Copyright (c) 2012 GarageGames, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

/*
   Resource stream -> Buffer
   Buffer -> Collada DOM
   Collada DOM -> TSShapeLoader
   TSShapeLoader installed into TSShape
*/

//-----------------------------------------------------------------------------

#include "platform/platform.h"

#include "ts/assimp/assimpShapeLoader.h"
#include "ts/assimp/assimpAppNode.h"
#include "ts/assimp/assimpAppMesh.h"
#include "ts/assimp/assimpAppMaterial.h"
#include "ts/assimp/assimpAppSequence.h"

#include "core/util/tVector.h"
#include "core/strings/findMatch.h"
#include "core/strings/stringUnit.h"
#include "core/stream/fileStream.h"
#include "core/fileObject.h"
#include "ts/tsShape.h"
#include "ts/tsShapeInstance.h"
#include "materials/materialManager.h"
#include "console/persistenceManager.h"
#include "ts/tsShapeConstruct.h"
#include "core/util/zip/zipVolume.h"
#include "gfx/bitmap/gBitmap.h"
#include "gui/controls/guiTreeViewCtrl.h"

// assimp include files. 
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/types.h>
#include <assimp/config.h>
#include <exception>

MODULE_BEGIN( AssimpShapeLoader )
   MODULE_INIT_AFTER( ShapeLoader )
   MODULE_INIT
   {
      TSShapeLoader::addFormat("DirectX X", "x");
      TSShapeLoader::addFormat("Autodesk FBX", "fbx");
      TSShapeLoader::addFormat("Blender 3D", "blend" );
      TSShapeLoader::addFormat("3ds Max 3DS", "3ds");
      TSShapeLoader::addFormat("3ds Max ASE", "ase");
      TSShapeLoader::addFormat("Wavefront Object", "obj");
      TSShapeLoader::addFormat("Industry Foundation Classes (IFC/Step)", "ifc");
      TSShapeLoader::addFormat("Stanford Polygon Library", "ply");
      TSShapeLoader::addFormat("AutoCAD DXF", "dxf");
      TSShapeLoader::addFormat("LightWave", "lwo");
      TSShapeLoader::addFormat("LightWave Scene", "lws");
      TSShapeLoader::addFormat("Modo", "lxo");
      TSShapeLoader::addFormat("Stereolithography", "stl");
      TSShapeLoader::addFormat("AC3D", "ac");
      TSShapeLoader::addFormat("Milkshape 3D", "ms3d");
      TSShapeLoader::addFormat("TrueSpace COB", "cob");
      TSShapeLoader::addFormat("TrueSpace SCN", "scn");
      TSShapeLoader::addFormat("Ogre XML", "xml");
      TSShapeLoader::addFormat("Irrlicht Mesh", "irrmesh");
      TSShapeLoader::addFormat("Irrlicht Scene", "irr");
      TSShapeLoader::addFormat("Quake I", "mdl" );
      TSShapeLoader::addFormat("Quake II", "md2" );
      TSShapeLoader::addFormat("Quake III Mesh", "md3");
      TSShapeLoader::addFormat("Quake III Map/BSP", "pk3");
      TSShapeLoader::addFormat("Return to Castle Wolfenstein", "mdc");
      TSShapeLoader::addFormat("Doom 3", "md5" );
      TSShapeLoader::addFormat("Valve SMD", "smd");
      TSShapeLoader::addFormat("Valve VTA", "vta");
      TSShapeLoader::addFormat("Starcraft II M3", "m3");
      TSShapeLoader::addFormat("Unreal", "3d");
      TSShapeLoader::addFormat("BlitzBasic 3D", "b3d" );
      TSShapeLoader::addFormat("Quick3D Q3D", "q3d");
      TSShapeLoader::addFormat("Quick3D Q3S", "q3s");
      TSShapeLoader::addFormat("Neutral File Format", "nff");
      TSShapeLoader::addFormat("Object File Format", "off");
      TSShapeLoader::addFormat("PovRAY Raw", "raw");
      TSShapeLoader::addFormat("Terragen Terrain", "ter");
      TSShapeLoader::addFormat("3D GameStudio (3DGS)", "mdl");
      TSShapeLoader::addFormat("3D GameStudio (3DGS) Terrain", "hmp");
      TSShapeLoader::addFormat("Izware Nendo", "ndo");
      TSShapeLoader::addFormat("gltf", "gltf");
      TSShapeLoader::addFormat("gltf binary", "glb");
   }
MODULE_END;

//-----------------------------------------------------------------------------

AssimpShapeLoader::AssimpShapeLoader() 
{
   mScene = NULL;
}

AssimpShapeLoader::~AssimpShapeLoader()
{

}

void AssimpShapeLoader::releaseImport()
{
}

void applyTransformation(aiNode* node, const aiMatrix4x4& transform) {
   node->mTransformation = transform * node->mTransformation; // Apply transformation to the node
}

void scaleScene(const aiScene* scene, F32 scaleFactor) {
   aiMatrix4x4 scaleMatrix;
   scaleMatrix = aiMatrix4x4::Scaling(aiVector3D(scaleFactor, scaleFactor, scaleFactor), scaleMatrix);
   applyTransformation(scene->mRootNode, scaleMatrix);
}

void debugSceneMetaData(const aiScene* scene) {
   if (!scene->mMetaData) {
      Con::printf("[ASSIMP] No metadata available.");
      return;
   }

   for (U32 i = 0; i < scene->mMetaData->mNumProperties; ++i) {
      const char* key = scene->mMetaData->mKeys[i].C_Str();
      aiMetadataType type = scene->mMetaData->mValues[i].mType;
      Con::printf("[ASSIMP] Metadata key: %s", key);

      switch (type) {
      case AI_BOOL:
         Con::printf("  Value: %d (bool)", *(bool*)scene->mMetaData->mValues[i].mData);
         break;
      case AI_INT32:
         Con::printf("  Value: %d (int)", *(S32*)scene->mMetaData->mValues[i].mData);
         break;
      case AI_UINT64:
         Con::printf("  Value: %llu (uint64)", *(U64*)scene->mMetaData->mValues[i].mData);
         break;
      case AI_FLOAT:
         Con::printf("  Value: %f (float)", *(F32*)scene->mMetaData->mValues[i].mData);
         break;
      case AI_DOUBLE:
         Con::printf("  Value: %f (double)", *(F64*)scene->mMetaData->mValues[i].mData);
         break;
      case AI_AISTRING:
         Con::printf("  Value: %s (string)", ((aiString*)scene->mMetaData->mValues[i].mData)->C_Str());
         break;
      case AI_AIVECTOR3D:
      {
         aiVector3D* vec = (aiVector3D*)scene->mMetaData->mValues[i].mData;
         Con::printf("  Value: (%f, %f, %f) (vector3d)", vec->x, vec->y, vec->z);
      }
      break;
      default:
         Con::printf("  Unknown metadata type.");
      }
   }
}

void AssimpShapeLoader::enumerateScene()
{
   TSShapeLoader::updateProgress(TSShapeLoader::Load_ReadFile, "Reading File");
   Con::printf("[ASSIMP] Attempting to load file: %s", shapePath.getFullPath().c_str());

   // Define post-processing steps
   U32 ppsteps = aiProcess_Triangulate | /*aiProcess_PreTransformVertices |*/ aiProcess_ConvertToLeftHanded & ~aiProcess_MakeLeftHanded;

   const auto& options = ColladaUtils::getOptions();
   if (options.calcTangentSpace) ppsteps |= aiProcess_CalcTangentSpace;
   if (options.joinIdenticalVerts) ppsteps |= aiProcess_JoinIdenticalVertices;
   if (options.removeRedundantMats) ppsteps |= aiProcess_RemoveRedundantMaterials;
   if (options.genUVCoords) ppsteps |= aiProcess_GenUVCoords;
   if (options.transformUVCoords) ppsteps |= aiProcess_TransformUVCoords;
   if (options.findInstances) ppsteps |= aiProcess_FindInstances;
   if (options.limitBoneWeights) ppsteps |= aiProcess_LimitBoneWeights;

   if (Con::getBoolVariable("$Assimp::OptimizeMeshes", false)) {
      ppsteps |= aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph;
   }
   if (Con::getBoolVariable("$Assimp::SplitLargeMeshes", false)) {
      ppsteps |= aiProcess_SplitLargeMeshes;
   }

   ppsteps |= aiProcess_ValidateDataStructure;

   struct aiLogStream shapeLog = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT, NULL);
   shapeLog.callback = assimpLogCallback;
   shapeLog.user = 0;
   aiAttachLogStream(&shapeLog);
#ifdef TORQUE_DEBUG
   aiEnableVerboseLogging(true);
#endif

   /*mImporter.SetPropertyInteger(AI_CONFIG_PP_PTV_KEEP_HIERARCHY, 1);
   mImporter.SetPropertyInteger(AI_CONFIG_PP_PTV_ADD_ROOT_TRANSFORMATION, 1);
   mImporter.SetPropertyMatrix(AI_CONFIG_PP_PTV_ROOT_TRANSFORMATION, aiMatrix4x4(1, 0, 0, 0,
                                                                                 0, 0, -1, 0,
                                                                                 0, 1, 0, 0,
                                                                                 0, 0, 0, 1));*/

   // Read the file
   mScene = mImporter.ReadFile(shapePath.getFullPath().c_str(), ppsteps);

   if (!mScene || (mScene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) || !mScene->mRootNode) {
      Con::errorf("[ASSIMP] ERROR: Could not load file: %s", shapePath.getFullPath().c_str());
      Con::errorf("[ASSIMP] Importer error: %s", mImporter.GetErrorString());
      TSShapeLoader::updateProgress(TSShapeLoader::Load_Complete, "Import failed");
      return;
   }

   Con::printf("[ASSIMP] Mesh Count: %d", mScene->mNumMeshes);
   Con::printf("[ASSIMP] Material Count: %d", mScene->mNumMaterials);

#ifdef TORQUE_DEBUG
   debugSceneMetaData(mScene);
#endif

   ColladaUtils::getOptions().upAxis = UPAXISTYPE_Y_UP; // default to Y up for assimp.
   // Handle scaling
   configureImportUnits();

   // Format-specific adjustments
   String fileExt = String::ToLower(shapePath.getExtension());
   const aiImporterDesc* importerDescription = aiGetImporterDesc(fileExt.c_str());
   if (importerDescription && dStrcmp(importerDescription->mName, "Autodesk FBX Importer") == 0) {
      Con::printf("[ASSIMP] Detected FBX format, checking unit scale...");
      F32 scaleFactor = ColladaUtils::getOptions().unit;
      if (scaleFactor != 1.0f) {
         Con::printf("[ASSIMP] Applying FBX scale factor: %f", scaleFactor);
         scaleScene(mScene, scaleFactor);
      }
      else
      {
         scaleScene(mScene, 0.01f);
      }
   }

   for (U32 i = 0; i < mScene->mNumTextures; ++i) {
      extractTexture(i, mScene->mTextures[i]);
   }

   // Load all materials
   AssimpAppMaterial::sDefaultMatNumber = 0;
   for (U32 i = 0; i < mScene->mNumMaterials; ++i) {
      AppMesh::appMaterials.push_back(new AssimpAppMaterial(mScene->mMaterials[i]));
   }

   // Setup LOD checks
   detectDetails();

   aiMatrix4x4 sceneRoot = aiMatrix4x4(1, 0, 0, 0,
      0, 0, -1, 0,
      0, 1, 0, 0,
      0, 0, 0, 1);

   applyTransformation(mScene->mRootNode, sceneRoot);

   // Process the scene graph
   AssimpAppNode* rootNode = new AssimpAppNode(mScene, mScene->mRootNode, 0);
   if (!processNode(rootNode)) {
      delete rootNode;
   }

   processAssimpNode(mScene->mRootNode, mScene, rootNode);

   // Add a bounds node if none exists
   if (!boundsNode) {
      aiNode* reqNode = new aiNode("bounds");
      mScene->mRootNode->addChildren(1, &reqNode);
      reqNode->mTransformation = aiMatrix4x4();// *sceneRoot;
      AssimpAppNode* appBoundsNode = new AssimpAppNode(mScene, reqNode);
      if (!processNode(appBoundsNode)) {
         delete appBoundsNode;
      }
   }

   // Process animations if available
   processAnimations();

   // Clean up log stream
   aiDetachLogStream(&shapeLog);
}

void AssimpShapeLoader::configureImportUnits() {
   auto& options = ColladaUtils::getOptions();

   // Configure unit scaling
   if (options.unit <= 0.0f) {
      F64 unitScaleFactor = 1.0;
      if (!getMetaDouble("UnitScaleFactor", unitScaleFactor)) {
         F32 floatVal;
         S32 intVal;
         if (getMetaFloat("UnitScaleFactor", floatVal)) {
            unitScaleFactor = static_cast<F64>(floatVal);
         }
         else if (getMetaInt("UnitScaleFactor", intVal)) {
            unitScaleFactor = static_cast<F64>(intVal);
         }
      }
      options.unit = static_cast<F32>(unitScaleFactor);
   }
}

void AssimpShapeLoader::processAssimpNode(const aiNode* node, const aiScene* scene, AssimpAppNode* parentNode)
{
   AssimpAppNode* currNode;
   if (node == scene->mRootNode)
   {
      currNode = parentNode;
   }
   else
   {
      currNode = new AssimpAppNode(scene, node, parentNode);

      if (parentNode)
      {
         parentNode->addChild(currNode);
      }

      for (U32 i = 0; i < node->mNumMeshes; i++)
      {
         U32 meshIdx = node->mMeshes[i];
         const aiMesh* mesh = scene->mMeshes[meshIdx];
         AssimpAppMesh* curMesh = new AssimpAppMesh(mesh, currNode);
         currNode->addMesh(curMesh);
      }
   }
   // Recursively process child nodes
   for (U32 i = 0; i < node->mNumChildren; i++)
   {
      processAssimpNode(node->mChildren[i], scene, currNode);
   }
}

void AssimpShapeLoader::processAnimations()
{
   // add all animations into 1 ambient animation.
   aiAnimation* ambientSeq = new aiAnimation();
   ambientSeq->mName = "ambient";

   Vector<aiNodeAnim*> ambientChannels;
   F32 duration = 0.0f;
   F32 ticks = 0.0f;
   if (mScene->mNumAnimations > 0)
   {
      for (U32 i = 0; i < mScene->mNumAnimations; ++i)
      {
         aiAnimation* anim = mScene->mAnimations[i];

         ticks = anim->mTicksPerSecond;

         duration = 0.0f;
         for (U32 j = 0; j < anim->mNumChannels; j++)
         {
            aiNodeAnim* nodeAnim = anim->mChannels[j];
            // Determine the maximum keyframe time for this animation
            F32 maxKeyTime = 0.0f;
            for (U32 k = 0; k < nodeAnim->mNumPositionKeys; k++) {
               maxKeyTime = getMax(maxKeyTime, (F32)nodeAnim->mPositionKeys[k].mTime);
            }
            for (U32 k = 0; k < nodeAnim->mNumRotationKeys; k++) {
               maxKeyTime = getMax(maxKeyTime, (F32)nodeAnim->mRotationKeys[k].mTime);
            }
            for (U32 k = 0; k < nodeAnim->mNumScalingKeys; k++) {
               maxKeyTime = getMax(maxKeyTime, (F32)nodeAnim->mScalingKeys[k].mTime);
            }

            ambientChannels.push_back(nodeAnim);

            duration = getMax(duration, maxKeyTime);
         }
      }

      ambientSeq->mNumChannels = ambientChannels.size();
      ambientSeq->mChannels = ambientChannels.address();
      ambientSeq->mDuration = duration;
      ambientSeq->mTicksPerSecond = ticks;

      AssimpAppSequence* defaultAssimpSeq = new AssimpAppSequence(ambientSeq);
      appSequences.push_back(defaultAssimpSeq);
   }
}

void AssimpShapeLoader::computeBounds(Box3F& bounds)
{
   TSShapeLoader::computeBounds(bounds);

   // Check if the model origin needs adjusting
   bool adjustCenter = ColladaUtils::getOptions().adjustCenter;
   bool adjustFloor = ColladaUtils::getOptions().adjustFloor;
   if (bounds.isValidBox() && (adjustCenter || adjustFloor))
   {
      // Compute shape offset
      Point3F shapeOffset = Point3F::Zero;
      if (adjustCenter)
      {
         bounds.getCenter(&shapeOffset);
         shapeOffset = -shapeOffset;
      }
      if (adjustFloor)
         shapeOffset.z = -bounds.minExtents.z;

      // Adjust bounds
      bounds.minExtents += shapeOffset;
      bounds.maxExtents += shapeOffset;

      // Now adjust all positions for root level nodes (nodes with no parent)
      for (S32 iNode = 0; iNode < shape->nodes.size(); iNode++)
      {
         if (!appNodes[iNode]->isParentRoot())
            continue;

         // Adjust default translation
         shape->defaultTranslations[iNode] += shapeOffset;

         // Adjust animated translations
         for (S32 iSeq = 0; iSeq < shape->sequences.size(); iSeq++)
         {
            const TSShape::Sequence& seq = shape->sequences[iSeq];
            if (seq.translationMatters.test(iNode))
            {
               for (S32 iFrame = 0; iFrame < seq.numKeyframes; iFrame++)
               {
                  S32 index = seq.baseTranslation + seq.translationMatters.count(iNode)*seq.numKeyframes + iFrame;
                  shape->nodeTranslations[index] += shapeOffset;
               }
            }
         }
      }
   }
}

bool AssimpShapeLoader::fillGuiTreeView(const char* sourceShapePath, GuiTreeViewCtrl* tree)
{
   Assimp::Importer importer;
   Torque::Path path(sourceShapePath);
   String cleanFile = AppMaterial::cleanString(path.getFileName());

   // Attempt to import with Assimp.
   const aiScene* shapeScene = importer.ReadFile(path.getFullPath().c_str(), (aiProcessPreset_TargetRealtime_Quality | aiProcess_CalcTangentSpace)
      & ~aiProcess_RemoveRedundantMaterials & ~aiProcess_GenSmoothNormals);

   if (!shapeScene)
   {
      Con::printf("AssimpShapeLoader::fillGuiTreeView - Assimp Error: %s", importer.GetErrorString());
      return false;
   }
   mScene = shapeScene;

   // Initialize tree
   tree->removeItem(0);
   S32 meshItem = tree->insertItem(0, "Meshes", String::ToString("%i", shapeScene->mNumMeshes));
   S32 matItem = tree->insertItem(0, "Materials", String::ToString("%i", shapeScene->mNumMaterials));
   S32 animItem = tree->insertItem(0, "Animations", String::ToString("%i", shapeScene->mNumAnimations));
   //S32 lightsItem = tree->insertItem(0, "Lights", String::ToString("%i", shapeScene->mNumLights));
   //S32 texturesItem = tree->insertItem(0, "Textures", String::ToString("%i", shapeScene->mNumTextures));

   //Details!
   U32 numPolys = 0;
   U32 numVerts = 0;
   for (U32 i = 0; i < shapeScene->mNumMeshes; i++)
   {
      tree->insertItem(meshItem, String::ToString("%s", shapeScene->mMeshes[i]->mName.C_Str()));
      numPolys += shapeScene->mMeshes[i]->mNumFaces;
      numVerts += shapeScene->mMeshes[i]->mNumVertices;
   }

   U32 defaultMatNumber = 0;
   for (U32 i = 0; i < shapeScene->mNumMaterials; i++)
   {
      aiMaterial* aiMat = shapeScene->mMaterials[i];

      aiString matName;
      aiMat->Get(AI_MATKEY_NAME, matName);
      String name = matName.C_Str();
      if (name.isEmpty())
      {
         name = AppMaterial::cleanString(path.getFileName());
         name += "_defMat";
         name += String::ToString("%d", defaultMatNumber);
         defaultMatNumber++;
      }

      aiString texPath;
      aiMat->GetTexture(aiTextureType::aiTextureType_DIFFUSE, 0, &texPath);
      String texName = texPath.C_Str();
      if (texName.isEmpty())
      {
         aiColor3D read_color(1.f, 1.f, 1.f);
         if (AI_SUCCESS == aiMat->Get(AI_MATKEY_COLOR_DIFFUSE, read_color))
            texName = String::ToString("Color: %0.3f %0.3f %0.3f", (F32)read_color.r, (F32)read_color.g, (F32)read_color.b); //formatted as words for easy parsing
         else
            texName = "No Texture";
      }
      else
         texName = AssimpAppMaterial::cleanTextureName(texName, cleanFile, sourceShapePath, true);

      tree->insertItem(matItem, String::ToString("%s", name.c_str()), String::ToString("%s", texName.c_str()));
   }

   if (shapeScene->mNumAnimations == 0)
   {
      tree->insertItem(animItem, "ambient", "animation", "", 0, 0);
   }
   else
   {
      for (U32 i = 0; i < shapeScene->mNumAnimations; i++)
      {
         tree->insertItem(animItem, shapeScene->mAnimations[i]->mName.C_Str(), "animation", "", 0, 0);
      }
   }

   U32 numNodes = 0;
   if (shapeScene->mRootNode)
   {
      S32 nodesItem = tree->insertItem(0, "Nodes", "");
      addNodeToTree(nodesItem, shapeScene->mRootNode, tree, numNodes);
      tree->setItemValue(nodesItem, String::ToString("%i", numNodes));
   }

   U32 numMetaTags = shapeScene->mMetaData ? shapeScene->mMetaData->mNumProperties : 0;
   if (numMetaTags)
      addMetaDataToTree(shapeScene->mMetaData, tree);

   F64 unit;
   if (!getMetaDouble("UnitScaleFactor", unit))
      unit = 1.0f;

   S32 upAxis;
   if (!getMetaInt("UpAxis", upAxis))
      upAxis = UPAXISTYPE_Z_UP;

   /*for (U32 i = 0; i < shapeScene->mNumLights; i++)
   {
      treeObj->insertItem(lightsItem, String::ToString("%s", shapeScene->mLights[i]->mType));
   }*/

   // Store shape information in the tree control
   tree->setDataField(StringTable->insert("_nodeCount"), 0, avar("%d", numNodes));
   tree->setDataField(StringTable->insert("_meshCount"), 0, avar("%d", shapeScene->mNumMeshes));
   tree->setDataField(StringTable->insert("_polygonCount"), 0, avar("%d", numPolys));
   tree->setDataField(StringTable->insert("_materialCount"), 0, avar("%d", shapeScene->mNumMaterials));
   tree->setDataField(StringTable->insert("_lightCount"), 0, avar("%d", shapeScene->mNumLights));
   tree->setDataField(StringTable->insert("_animCount"), 0, avar("%d", shapeScene->mNumAnimations));
   tree->setDataField(StringTable->insert("_textureCount"), 0, avar("%d", shapeScene->mNumTextures));
   tree->setDataField(StringTable->insert("_vertCount"), 0, avar("%d", numVerts));
   tree->setDataField(StringTable->insert("_metaTagCount"), 0, avar("%d", numMetaTags));
   tree->setDataField(StringTable->insert("_unit"), 0, avar("%g", (F32)unit));

   if (upAxis == UPAXISTYPE_X_UP)
      tree->setDataField(StringTable->insert("_upAxis"), 0, "X_AXIS");
   else if (upAxis == UPAXISTYPE_Y_UP)
      tree->setDataField(StringTable->insert("_upAxis"), 0, "Y_AXIS");
   else
      tree->setDataField(StringTable->insert("_upAxis"), 0, "Z_AXIS");

   return true;
}

void AssimpShapeLoader::updateMaterialsScript(const Torque::Path &path)
{
   return;
   /*
   Torque::Path scriptPath(path);
   scriptPath.setFileName("materials");
   scriptPath.setExtension(TORQUE_SCRIPT_EXTENSION);

   // First see what materials we need to update
   PersistenceManager persistMgr;
   for ( U32 iMat = 0; iMat < AppMesh::appMaterials.size(); iMat++ )
   {
      AssimpAppMaterial *mat = dynamic_cast<AssimpAppMaterial*>( AppMesh::appMaterials[iMat] );
      if ( mat )
      {
         Material *mappedMat;
         if ( Sim::findObject( MATMGR->getMapEntry( mat->getName() ), mappedMat ) )
         {
            // Only update existing materials if forced to
            if (ColladaUtils::getOptions().forceUpdateMaterials)
            {
               mat->initMaterial(scriptPath, mappedMat);
               persistMgr.setDirty(mappedMat);
            }
         }
         else
         {
            // Create a new material definition
            persistMgr.setDirty( mat->createMaterial( scriptPath ), scriptPath.getFullPath() );
         }
      }
   }

   if ( persistMgr.getDirtyList().empty() )
      return;

   persistMgr.saveDirty();
   */
}

/// Check if an up-to-date cached DTS is available for this DAE file
bool AssimpShapeLoader::canLoadCachedDTS(const Torque::Path& path)
{
   // Generate the cached filename
   Torque::Path cachedPath(path);
   cachedPath.setExtension("cached.dts");

   // Check if a cached DTS newer than this file is available
   FileTime cachedModifyTime;
   if (Platform::getFileTimes(cachedPath.getFullPath(), NULL, &cachedModifyTime))
   {
      bool forceLoad = Con::getBoolVariable("$assimp::forceLoad", false);

      FileTime daeModifyTime;
      if (!Platform::getFileTimes(path.getFullPath(), NULL, &daeModifyTime) ||
         (!forceLoad && (Platform::compareFileTimes(cachedModifyTime, daeModifyTime) >= 0) ))
      {
         // Original file not found, or cached DTS is newer
         return true;
      }
   }

   return false;
}

void AssimpShapeLoader::assimpLogCallback(const char* message, char* user)
{
   Con::printf("[Assimp log message] %s", StringUnit::getUnit(message, 0, "\n"));
}

bool AssimpShapeLoader::ignoreNode(const String& name)
{
   // Do not add AssimpFbx dummy nodes to the TSShape. See: Assimp::FBX::ImportSettings::preservePivots
   // https://github.com/assimp/assimp/blob/master/code/FBXImportSettings.h#L116-L135
   if (name.find("_$AssimpFbx$_") != String::NPos)
      return true;

   if (FindMatch::isMatchMultipleExprs(ColladaUtils::getOptions().alwaysImport, name, false))
      return false;

   return FindMatch::isMatchMultipleExprs(ColladaUtils::getOptions().neverImport, name, false);
}

bool AssimpShapeLoader::ignoreMesh(const String& name)
{
   if (FindMatch::isMatchMultipleExprs(ColladaUtils::getOptions().alwaysImportMesh, name, false))
      return false;
   else
      return FindMatch::isMatchMultipleExprs(ColladaUtils::getOptions().neverImportMesh, name, false);
}

void AssimpShapeLoader::detectDetails()
{
   // Set LOD option
   bool singleDetail = true;
   switch (ColladaUtils::getOptions().lodType)
   {
   case ColladaUtils::ImportOptions::DetectDTS:
      // Check for a baseXX->startXX hierarchy at the top-level, if we find
      // one, use trailing numbers for LOD, otherwise use a single size
      for (S32 iNode = 0; singleDetail && (iNode < mScene->mRootNode->mNumChildren); iNode++) {
         aiNode* node = mScene->mRootNode->mChildren[iNode];
         if (node && dStrStartsWith(node->mName.C_Str(), "base")) {
            for (S32 iChild = 0; iChild < node->mNumChildren; iChild++) {
               aiNode* child = node->mChildren[iChild];
               if (child && dStrStartsWith(child->mName.C_Str(), "start")) {
                  singleDetail = false;
                  break;
               }
            }
         }
      }
      break;

   case ColladaUtils::ImportOptions::SingleSize:
      singleDetail = true;
      break;

   case ColladaUtils::ImportOptions::TrailingNumber:
      singleDetail = false;
      break;

   default:
      break;
   }

   AssimpAppMesh::fixDetailSize(singleDetail, ColladaUtils::getOptions().singleDetailSize);
}

void AssimpShapeLoader::extractTexture(U32 index, aiTexture* pTex)
{  // Cache an embedded texture to disk
   updateProgress(Load_EnumerateScene, "Extracting Textures...", mScene->mNumTextures, index);
   Con::printf("[Assimp] Extracting Texture %s, W: %d, H: %d, %d of %d, format hint: (%s)", pTex->mFilename.C_Str(),
      pTex->mWidth, pTex->mHeight, index, mScene->mNumTextures, pTex->achFormatHint);

   // Create the texture filename
   String cleanFile = AppMaterial::cleanString(TSShapeLoader::getShapePath().getFileName());
   String texName = String::ToString("%s_cachedTex%d", cleanFile.c_str(), index);
   Torque::Path texPath = shapePath;
   texPath.setFileName(texName);

   if (pTex->mHeight == 0)
   {  // Compressed format, write the data directly to disc
      texPath.setExtension(pTex->achFormatHint);
      FileStream *outputStream;
      if ((outputStream = FileStream::createAndOpen(texPath.getFullPath(), Torque::FS::File::Write)) != NULL)
      {
         outputStream->setPosition(0);
         outputStream->write(pTex->mWidth, pTex->pcData);
         outputStream->close();
         delete outputStream;
      }
   }
   else
   {  // Embedded pixel data, fill a bitmap and save it.
      GFXTexHandle shapeTex;
      shapeTex.set(pTex->mWidth, pTex->mHeight, GFXFormatR8G8B8A8_SRGB, &GFXDynamicTextureSRGBProfile,
         String::ToString("AssimpShapeLoader (%s:%i)", __FILE__, __LINE__), 1, 0);
      GFXLockedRect *rect = shapeTex.lock();

      for (U32 y = 0; y < pTex->mHeight; ++y)
      {
         for (U32 x = 0; x < pTex->mWidth; ++x)
         {
            U32 targetIndex = (y * rect->pitch) + (x * 4);
            U32 sourceIndex = ((y * pTex->mWidth) + x) * 4;
            rect->bits[targetIndex] = pTex->pcData[sourceIndex].r;
            rect->bits[targetIndex + 1] = pTex->pcData[sourceIndex].g;
            rect->bits[targetIndex + 2] = pTex->pcData[sourceIndex].b;
            rect->bits[targetIndex + 3] = pTex->pcData[sourceIndex].a;
         }
      }
      shapeTex.unlock();

      texPath.setExtension("png");
      shapeTex->dumpToDisk("PNG", texPath.getFullPath());
   }
}

void AssimpShapeLoader::addNodeToTree(S32 parentItem, aiNode* node, GuiTreeViewCtrl* tree, U32& nodeCount)
{
   // Add this node
   S32 nodeItem = parentItem;
   String nodeName = node->mName.C_Str();
   if (!ignoreNode(nodeName))
   {
      if (nodeName.isEmpty())
         nodeName = "null";
      nodeItem = tree->insertItem(parentItem, nodeName.c_str(), String::ToString("%i", node->mNumChildren));
      nodeCount++;
   }

   // Add any child nodes
   for (U32 n = 0; n < node->mNumChildren; ++n)
      addNodeToTree(nodeItem, node->mChildren[n], tree, nodeCount);
}

void AssimpShapeLoader::addMetaDataToTree(const aiMetadata* metaData, GuiTreeViewCtrl* tree)
{
   S32 metaItem = tree->insertItem(0, "MetaData", String::ToString("%i", metaData->mNumProperties));

   aiString valString;
   aiVector3D valVec;

   for (U32 n = 0; n < metaData->mNumProperties; ++n)
   {
      String keyStr = metaData->mKeys[n].C_Str();
      keyStr += ": ";
      switch (metaData->mValues[n].mType)
      {
      case AI_BOOL:
         keyStr += ((bool)metaData->mValues[n].mData) ? "true" : "false";
         break;
      case AI_INT32:
         keyStr += String::ToString(*((S32*)(metaData->mValues[n].mData)));
         break;
      case AI_UINT64:
         keyStr += String::ToString("%I64u", *((U64*)metaData->mValues[n].mData));
         break;
      case AI_FLOAT:
         keyStr += String::ToString(*((F32*)metaData->mValues[n].mData));
         break;
      case AI_DOUBLE:
         keyStr += String::ToString(*((F64*)metaData->mValues[n].mData));
         break;
      case AI_AISTRING:
         metaData->Get<aiString>(metaData->mKeys[n], valString);
         keyStr += valString.C_Str();
         break;
      case AI_AIVECTOR3D:
         metaData->Get<aiVector3D>(metaData->mKeys[n], valVec);
         keyStr += String::ToString("%f, %f, %f", valVec.x, valVec.y, valVec.z);
         break;
      default:
         break;
      }

      tree->insertItem(metaItem, keyStr.c_str(), String::ToString("%i", n));
   }
}

bool AssimpShapeLoader::getMetabool(const char* key, bool& boolVal)
{
   if (!mScene || !mScene->mMetaData)
      return false;

   String keyStr = key;
   for (U32 n = 0; n < mScene->mMetaData->mNumProperties; ++n)
   {
      if (keyStr.equal(mScene->mMetaData->mKeys[n].C_Str(), String::NoCase))
      {
         if (mScene->mMetaData->mValues[n].mType == AI_BOOL)
         {
            boolVal = (bool)mScene->mMetaData->mValues[n].mData;
            return true;
         }
      }
   }
   return false;
}

bool AssimpShapeLoader::getMetaInt(const char* key, S32& intVal)
{
   if (!mScene || !mScene->mMetaData)
      return false;

   String keyStr = key;
   for (U32 n = 0; n < mScene->mMetaData->mNumProperties; ++n)
   {
      if (keyStr.equal(mScene->mMetaData->mKeys[n].C_Str(), String::NoCase))
      {
         if (mScene->mMetaData->mValues[n].mType == AI_INT32)
         {
            intVal = *((S32*)(mScene->mMetaData->mValues[n].mData));
            return true;
         }
      }
   }
   return false;
}

bool AssimpShapeLoader::getMetaFloat(const char* key, F32& floatVal)
{
   if (!mScene || !mScene->mMetaData)
      return false;

   String keyStr = key;
   for (U32 n = 0; n < mScene->mMetaData->mNumProperties; ++n)
   {
      if (keyStr.equal(mScene->mMetaData->mKeys[n].C_Str(), String::NoCase))
      {
         if (mScene->mMetaData->mValues[n].mType == AI_FLOAT)
         {
            floatVal = *((F32*)mScene->mMetaData->mValues[n].mData);
            return true;
         }
      }
   }
   return false;
}

bool AssimpShapeLoader::getMetaDouble(const char* key, F64& doubleVal)
{
   if (!mScene || !mScene->mMetaData)
      return false;

   String keyStr = key;
   for (U32 n = 0; n < mScene->mMetaData->mNumProperties; ++n)
   {
      if (keyStr.equal(mScene->mMetaData->mKeys[n].C_Str(), String::NoCase))
      {
         if (mScene->mMetaData->mValues[n].mType == AI_DOUBLE)
         {
            doubleVal = *((F64*)mScene->mMetaData->mValues[n].mData);
            return true;
         }
      }
   }
   return false;
}

bool AssimpShapeLoader::getMetaString(const char* key, String& stringVal)
{
   if (!mScene || !mScene->mMetaData)
      return false;

   String keyStr = key;
   for (U32 n = 0; n < mScene->mMetaData->mNumProperties; ++n)
   {
      if (keyStr.equal(mScene->mMetaData->mKeys[n].C_Str(), String::NoCase))
      {
         if (mScene->mMetaData->mValues[n].mType == AI_AISTRING)
         {
            aiString valString;
            mScene->mMetaData->Get<aiString>(mScene->mMetaData->mKeys[n], valString);
            stringVal = valString.C_Str();
            return true;
         }
      }
   }
   return false;
}
//-----------------------------------------------------------------------------
/// This function is invoked by the resource manager based on file extension.
TSShape* assimpLoadShape(const Torque::Path &path)
{
   // TODO: add .cached.dts generation.
   // Generate the cached filename
   Torque::Path cachedPath(path);
   cachedPath.setExtension("cached.dts");

   // Check if an up-to-date cached DTS version of this file exists, and
   // if so, use that instead.
   if (AssimpShapeLoader::canLoadCachedDTS(path))
   {
      FileStream cachedStream;
      cachedStream.open(cachedPath.getFullPath(), Torque::FS::File::Read);
      if (cachedStream.getStatus() == Stream::Ok)
      {
         TSShape *shape = new TSShape;
         bool readSuccess = shape->read(&cachedStream);
         cachedStream.close();

         if (readSuccess)
         {
         #ifdef TORQUE_DEBUG
            Con::printf("Loaded cached shape from %s", cachedPath.getFullPath().c_str());
         #endif
            return shape;
         }
         else
            delete shape;
      }

      Con::warnf("Failed to load cached shape from %s", cachedPath.getFullPath().c_str());
   }

   if (!Torque::FS::IsFile(path))
   {
      // File does not exist, bail.
      return NULL;
   }

   // Allow TSShapeConstructor object to override properties
   ColladaUtils::getOptions().reset();
   TSShapeConstructor* tscon = TSShapeConstructor::findShapeConstructorByFilename(path.getFullPath());
   if (tscon)
   {
      ColladaUtils::getOptions() = tscon->mOptions;
   }

   AssimpShapeLoader loader;
   TSShape* tss = loader.generateShape(path);
   if (tss)
   {
      TSShapeLoader::updateProgress(TSShapeLoader::Load_Complete, "Import complete");
      Con::printf("[ASSIMP] Shape created successfully.");

      // Cache the model to a DTS file for faster loading next time.
      FileStream dtsStream;
      if (dtsStream.open(cachedPath.getFullPath(), Torque::FS::File::Write))
      {
         Con::printf("Writing cached shape to %s", cachedPath.getFullPath().c_str());
         tss->write(&dtsStream);
      }

      loader.updateMaterialsScript(path);
   }
   loader.releaseImport();
   return tss;
}

DefineEngineFunction(GetShapeInfo, bool, (const char* shapePath, const char* ctrl, bool loadCachedDts), ("", "", true),
   "(string shapePath, GuiTreeViewCtrl ctrl) Collect scene information from "
   "a shape file and store it in a GuiTreeView control. This function is "
   "used by the assimp import gui to show a preview of the scene contents "
   "prior to import, and is probably not much use for anything else.\n"
   "@param shapePath shape filename\n"
   "@param ctrl GuiTreeView control to add elements to\n"
   "@return true if successful, false otherwise\n"
   "@ingroup Editors\n"
   "@internal")
{
   GuiTreeViewCtrl* tree;
   if (!Sim::findObject(ctrl, tree))
   {
      Con::errorf("enumColladaScene::Could not find GuiTreeViewCtrl '%s'", ctrl);
      return false;
   }

   // Check if a cached DTS is available => no need to import the source file
   // if we can load the DTS instead
   Torque::Path path(shapePath);
   if (loadCachedDts && AssimpShapeLoader::canLoadCachedDTS(path))
      return false;

   AssimpShapeLoader loader;
   return loader.fillGuiTreeView(shapePath, tree);
}
