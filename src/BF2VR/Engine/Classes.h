// Copyright Ethan Porcaro

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#pragma once

#include <Offsets.h>
#include <windows.h>
#include <d3d11.h>
#include <cstdint>
#include <Types.h>
#include <safetyhook.hpp>


///////////////////////////////////
// Utils
///////////////////////////////////

typedef Vec3 Vector3;

static inline bool isValidPtr(PVOID p) {
    return (p >= (PVOID)0x10000) && (p < ((PVOID)0x000F000000000000)) && p != nullptr &&
        !IsBadReadPtr(p, sizeof(PVOID));
}

///////////////////////////////////
// Rendering
///////////////////////////////////

class Screen {
 public:
    char pad_0000[80];  //  0x0000
    uint32_t bufferWidth;  //  0x0050
    uint32_t bufferHeight;  //  0x0054
    uint32_t anotherWidth;  //  0x0058
    uint32_t anotherHeight;  //  0x005C
};

class DXRenderer {
 public:
    static ID3D11Device* getDevice() {
        DWORD64 offset1 = *reinterpret_cast<DWORD64*>(OFFSETDXRENDERER);
        return *reinterpret_cast<ID3D11Device**>(offset1 + 0xE98);
    }
    static ID3D11DeviceContext* getContext() {
        DWORD64 offset1 = *reinterpret_cast<DWORD64*>(OFFSETDXRENDERER);
        return *reinterpret_cast<ID3D11DeviceContext**>(offset1 + 0xEA0);
    }
    static IDXGISwapChain* getSwapChain() {
        DWORD64 offset1 = *reinterpret_cast<DWORD64*>(OFFSETDXRENDERER);
        return *reinterpret_cast<IDXGISwapChain**>(offset1 + 0x338);
    }
    static Screen* getScreen() {
        DWORD64 offset1 = *reinterpret_cast<DWORD64*>(OFFSETDXRENDERER);
        return *reinterpret_cast<Screen**>(offset1 + 0xC88);
    }
};

class RenderView {
 public:
    Matrix4 transform;  // 0x0000
    char pad_0040[112];  // 0x0040
    float fov;  // 0x00B0
    char pad_00B4[20];  // 0x00B4
    float nearPlane;  // 0x00C8
    float farPlane;  // 0x00CC
    float aspectRatio;  // 0x00D0
    float orthoWidth;  // 0x00D4
    float orthoHeight;  // 0x00D8
    char pad_00DC[404];  // 0x00DC
    Matrix4 viewMatrix;  // 0x0270
    Matrix4 viewMatrixTranspose;  // 0x02B0
    Matrix4 viewMatrixInverse;  // 0x02F0
    Matrix4 projectionMatrix;  // 0x0330
    Matrix4 viewMatrixAtOrigin;  // 0x0370
    Matrix4 projectionTranspose;  // 0x03B0
    Matrix4 projectionInverse;  // 0x03F0
    Matrix4 viewProjection;  // 0x0430
    Matrix4 viewProjectionTranspose;  // 0x0470
    Matrix4 viewProjectionInverse;  // 0x04B0
};

class Skybox {
 public:
     char pad_0000[712];  // 0x0000
     bool enable;  // 0x02C8

    static Skybox* GetInstance() {
        __int64 off1 = *reinterpret_cast<__int64*>(OFFSETSKYBOX);
        if (!isValidPtr(reinterpret_cast<void*>(off1))) {
            return nullptr;
        }

        __int64 off2 = *reinterpret_cast<__int64*>(off1 + 0xc0);
        if (!isValidPtr(reinterpret_cast<void*>(off2))) {
            return nullptr;
        }

        __int64 off3 = *reinterpret_cast<__int64*>(off2 + 0x5e8);
        if (!isValidPtr(reinterpret_cast<void*>(off3))) {
            return nullptr;
        }

        Skybox* skybox = *reinterpret_cast<Skybox**>(off3 + 0x138);
        if (!isValidPtr(skybox)) {
            return nullptr;
        }
        return skybox;
    }
};
// I use structs cause I don't like declaring stuff public every time. Pet peev of mine
struct GameRenderSettings {
    uint32_t InactiveSkipFrameCount;
    float ResolutionScale;
    float ResolutionScaleMax;
    int32_t MantleEnable;
    float CameraCutMaxFrameTranslation;
    float NearPlane;
    float ViewDistance;
    float ForceFov;
    float FovMultiplier;
    float ForceOrthoViewSize;
    float EdgeModelScreenAreaScale;
    float EdgeModelViewDistance;
    int32_t EdgeModelForceLod;
    float EdgeModelLodScale;
    float StaticModelPartOcclusionMaxScreenArea;
    uint32_t StaticModelCullJobCount;
    uint32_t SplitScreenTestViewCount;
    uint32_t SplitScreenTestCase;
    float ForceBlurAmount;
    float ForceWorldFadeAmount;
    float StereoCrosshairMaxHitDepth;
    float StereoCrosshairRadius;
    float StereoCrosshairDampingFactor;
    uint32_t UIBlurTextureDivisor;
    uint32_t UIBlurFilter;
    float UIBlurDeviation;
    bool Enable;
    bool NullRendererEnable;
    bool JobEnable;
    bool BuildJobSyncEnable;
    bool DrawDebugDynamicTextureArrays;
    bool DrawDebugInfo;
    bool DrawScreenInfo;
    bool ResolutionScaleDynamicEnabled;
    bool Fullscreen;
    bool ForceVSyncEnable;
    bool MovieVSyncEnable;
    bool VSyncFlashTestEnable;
    bool OutputBrightnessTestEnable;
    bool GlEnable;
    bool Dx11Enable;
    bool Dx12Enable;
    bool BalsaEnable;
    bool Gen4bColorRemap;
    bool GpuTextureCompressorEnable;
    bool MeshWorldEnable;
    bool EmittersEnable;
    bool EntityRenderEnable;
    bool DebugRendererEnable;
    bool DebugRenderServiceEnable;
    bool InitialClearEnable;
    bool GpuProfilerEnable;
    bool ForceOrthoViewEnable;
    bool ForceSquareOrthoView;
    bool DestructionVolumeDrawEnable;
    bool EdgeModelsEnable;
    bool EdgeModelCastShadowsEnable;
    bool EdgeModelDepthBiasEnable;
    bool EdgeModelShadowDepthBiasEnable;
    bool EdgeModelUseMainLodEnable;
    bool EdgeModelUseLodBox;
    bool EdgeModelCullEnable;
    bool EdgeModelFrustumCullEnable;
    bool EdgeModelDrawBoxes;
    bool EdgeModelDrawStats;
    bool StaticModelEnable;
    bool StaticModelMeshesEnable;
    bool StaticModelZPassEnable;
    bool StaticModelPartCullEnable;
    bool StaticModelPartFrustumCullEnable;
    bool StaticModelPartOcclusionCullEnable;
    bool StaticModelPartShadowCullEnable;
    bool StaticModelDrawBoxes;
    bool StaticModelDrawStats;
    bool StaticModelCullSpuJobEnable;
    bool StaticModelSurfaceShaderTerrainAccessEnable;
    bool LockView;
    bool ResetLockedView;
    bool InfiniteProjectionMatrixEnable;
    bool SecondaryStreamingViewEnable;
    bool FadeEnable;
    bool FadeWaitingEnable;
    bool BlurEnable;
};

class GameRenderer {
 public:
    char pad_0000[1304];  // 0x0000
    class GameRenderSettings* gameRenderSettings;  // 0x0510
    char pad_0520[24];  // 0x0520
    class RenderView* renderView;  // 0x0538
    char pad_0540[4872];  // 0x0540

    static GameRenderer* GetInstance() {
        return *reinterpret_cast<GameRenderer**>(OFFSETGAMERENDERER);
    }
};
enum QualityLevel
{
    QualityLevel_Low,
    QualityLevel_Medium,
    QualityLevel_High,
    QualityLevel_Ultra,
    QualityLevel_All,
    QualityLevel_Invalid,
};
enum LightTileDebugLightCountMode
{
    LightTileDebugLightCountMode_Total,
    LightTileDebugLightCountMode_Punctual,
    LightTileDebugLightCountMode_PunctualShadow,
    LightTileDebugLightCountMode_Area,
    LightTileDebugLightCountMode_AreaShadow,
    LightTileDebugLightCountMode_LocalIBL,
    LightTileDebugLightCountMode_LocalPR,
};
enum PostProcessAAMode
{
    PostProcessAAMode_None,
    PostProcessAAMode_FxaaLow,
    PostProcessAAMode_FxaaMedium,
    PostProcessAAMode_FxaaHigh,
    PostProcessAAMode_FxaaCompute,
    PostProcessAAMode_FxaaComputeExtreme,
    PostProcessAAMode_Smaa1x,
    PostProcessAAMode_SmaaT2x,
    PostProcessAAMode_TemporalAA,
};
enum ScaleResampleMode
{
    ScaleResampleMode_Point,
    ScaleResampleMode_Linear,
    ScaleResampleMode_Bicubic,
    ScaleResampleMode_Lanczos,
    ScaleResampleMode_LanczosSeparable,
    ScaleResampleMode_BicubicSharp,
    ScaleResampleMode_BicubicSharpSeparable,
};
enum SkyRenderMode
{
    SkyRenderMode_SkyBox,
    SkyRenderMode_PhysicallyBased,
};
enum SpotLightShadowmapTextureMode
{
    SpotLightShadowmapTextureMode_Normal,
    SpotLightShadowmapTextureMode_Array,
    SpotLightShadowmapTextureMode_Atlas,
};
struct QualityScalableInt
{
    int32_t Low;
    int32_t Medium;
    int32_t High;
    int32_t Ultra;
};
struct WorldRenderSettingsBase
{
    float CullScreenAreaScale;
    float MinShadowViewCoverage;
    Vec3 MotionBlurClearColor;
    Vec3 DynamicEnvmapDefaultPosition;
    float ShadowmapMinFov;
    float ShadowmapSizeZScale;
    uint32_t ShadowmapResolution;
    uint32_t ShadowmapResolutionMax;
    uint32_t ShadowmapQuality;
    float ShadowmapPoissonFilterScale;
    uint32_t ShadowmapSliceCount;
    float ShadowmapSliceSchemeWeight;
    float ShadowmapFirstSliceScale;
    float ShadowmapViewDistance;
    float ShadowmapExtrusionLength;
    float ForceShadowmapFirstSliceViewDistance;
    float CockpitShadowsExtrusionLength;
    float ShadowmapTransitionBlendAmount;
    float ShadowmapForegroundExtrusionLength;
    float ShadowmapForegroundSplitDistance;
    float ShadowmapForegroundSizeZScale;
    float ViewportMaskScale;
    float ViewportMaskInnerScale;
    int32_t SunPcssMaxSampleCount;
    int32_t SunPcssAdaptiveSampleIncrement;
    float MotionBlurScale;
    float MotionBlurFixedShutterTime;
    float MotionBlurMax;
    float MotionBlurRadialBlurMax;
    float MotionBlurNoiseScale;
    uint32_t MotionBlurQuality;
    uint32_t MotionBlurDebugMode;
    uint32_t MotionBlurMaxSampleCount;
    float ForceMotionBlurDepthCutoff;
    float ForceMotionBlurCutoffGradientScale;
    float MotionBlurDepthCheckThreshold;
    float MotionBlurDepthCheckMaxDistance;
    uint32_t MultisampleCount;
    uint32_t MultisampleQuality;
    int32_t OnlyShadowmapSlice;
    char padding[4];                //WorldViewMode ViewMode;
    uint32_t AdditionalHdrTargetInESRAM;
    int32_t DrawDebugBlurPyramidMipLevel;
    uint32_t DrawDebugBuffers;
    float HalfResDepthMinMaxDitherThreshold;
    uint32_t PhysicalSkyPrecisionHeight;
    uint32_t PhysicalSkyPrecisionView;
    uint32_t PhysicalSkyPrecisionSun;
    uint32_t PhysicalSkyScatteringOrders;
    uint32_t PhysicalSkyAerialPerspectiveTextureWidth;
    uint32_t PhysicalSkyAerialPerspectiveTextureHeight;
    uint32_t PhysicalSkyAerialPerspectiveTextureDepth;
    uint32_t PhysicalSkyScatteringEvalFrameCount;
    float PhysicalSkyAerialPerspectiveMaxDistance;
    float SkyEnvmapFilterWidth;
    uint32_t SkyEnvmapResolution;
    int32_t DrawDebugSkyEnvmapMipLevel;
    char padding2[4];                //MipmapFilterMode SkyEnvmapFilterMode;
    uint32_t SkyEnvmapSidesPerFrameCount;
    float SkyEnvmapUpdateCountThreshold;
    float SkyEnvmapUpdateValueThreshold;
    float DynamicEnvmapFilterWidth;
    int32_t DrawDebugDynamicEnvmapMipLevel;
    char padding3[4];               //MipmapFilterMode DynamicEnvmapFilterMode;
    uint32_t DynamicEnvmapSpecularConvolutionSampleCount;
    uint32_t DynamicEnvmapShadowmapResolution;
    int32_t DynamicEnvmapShadowmapFarPlane;
    int32_t DynamicEnvmapShadowmapShadowExtrusion;
    bool DeferredShadingEnable;
    bool ForwardOpaqueEnable;
    bool FullZPassEnable;
    bool TileMaterialClassificationEnable;
    bool ShadowmapsEnable;
    bool ShadowmapArrayEnable;
    bool TransparencyShadowmapsEnable;
    bool TransparencyShadowmapsHalfRes;
    bool ShadowmapFixedMovementEnable;
    bool ShadowmapFixedDepthEnable;
    bool ShadowmapViewDistanceScaleEnable;
    bool ShadowmapCullVolumeEnable;
    bool ShadowmapAdjustFarPlane;
    bool CockpitShadowsEnable;
    bool ShadowmapAccumEnable;
    bool ShadowmapAccumReuseEnable;
    bool ShadowmapAccumBilinearEnable;
    bool ShadowmapAccumStencilEnable;
    bool ShadowmapAccumStencil2Enable;
    bool ShadowmapTransitionBlendEnable;
    bool ShadowmapForegroundEnable;
    bool ShadowmapForegroundUseFirstPersonViewTransform;
    bool ShadowmapStereoSharedEnable;
    bool DrawListStereoSharedEnable;
    bool DxShadowmap16BitEnable;
    bool DxSpotLightShadowmap16BitEnable;
    bool DxDynamicEnvmapShadowmap16BitEnable;
    bool ApplyShadowmapsEnable;
    bool GenerateShadowmapsEnable;
    bool SimpleShadowmapsEnable;
    bool EmitterShadowingBlendToggle;
    bool EmitterShadowingManySamplesToggle;
    bool DxLinearDepth32BitFormatEnable;
    bool MotionBlurEnable;
    bool MotionBlurForceOn;
    bool MotionBlurOptimalStableVelocityFormula;
    bool MotionBlurPreciseStableVelocityFormula;
    bool MotionBlurStencilPassEnable;
    bool MotionBlurGeometryPassEnable;
    bool MotionBlurBackgroundPassEnable;
    bool MotionBlurCenteredEnable;
    bool MotionBlurHairPassEnable;
    bool DrawTransparent;
    bool DrawTransparentDecal;
    bool TransparentAfterMotionBlur;
    bool Enable;
    bool HdrEnable;
    bool LdrEnable;
    bool ReadOnlyDepthEnable;
    bool ConsoleRenderTargetPoolSharingEnable;
    bool FastHdrEnable;
    bool LinearDepthInESRAM;
    bool HalfResDepthResolveEnable;
    bool FinalPostEnable;
    bool OutputGammaCorrectionEnable;
    bool ScreenEffectEnable;
    bool DrawSolidBoundingBoxes;
    bool DrawLineBoundingBoxes;
    bool DrawBoundingSpheres;
    bool DrawFrustums;
    bool DrawLocalIBLFrustums;
    bool DrawDebugShadowmaps;
    bool DrawDebugSpotLightShadowmaps;
    bool DrawDebugSkyEnvmap;
    bool DrawDebugVelocityBuffer;
    bool DrawDebugZBufferEnable;
    bool DrawDebugHalfResEnvironment;
    bool DrawDebugDistortion;
    bool DrawDebugVisibleEntityTypes;
    bool DrawDebugSkyTextures;
    bool DrawDebugMarschnerTextures;
    bool DrawDebugDof;
    bool DrawDebugDofFullscreen;
    bool DrawDebugHalfResHdrTargets;
    bool DrawDebugHiZMinMaxBufferEnable;
    bool DrawDebugScreenSpaceRaytraceBucketsEnable;
    bool DrawDebugEmitterSunTransmittanceMaps;
    bool DrawDebugBlurPyramid;
    bool DrawDebugOcclusionZBuffer;
    bool DrawDebugLocalIBLOcclusionZBuffer;
    bool WireframeEnable;
    bool ZPassEnable;
    bool OccluderMeshZPrepassEnable;
    bool OccluderMeshZPrepassDrawEnable;
    bool OccluderMeshZPrepassDebugEnable;
    bool HalfResEnable;
    bool ForceFullResEnable;
    bool HalfResLensFlaresEnable;
    bool ForegroundEnable;
    bool ForegroundDepthClearEnable;
    bool ForegroundZPassEnable;
    bool ForegroundTransparentEnable;
    bool ThirdPersonFriendlySSREnable;
    bool MidgroundEnable;
    bool ExtraHalfResDepthForSSAO;
    bool BilateralHalfResCompositeEnable;
    bool HalfResDepthMinMaxDitherEnable;
    bool SkyLightingEnable;
    bool SkyRenderEnable;
    bool SkyDepthFogEnable;
    bool SkyHeightFogEnable;
    bool SkyForwardScatteringEnable;
    bool PhysicalSkyEnabled;
    bool PhysicalSkyForcePrecompute;
    bool TransparentFoggingEnable;
    bool DistortionEnable;
    bool DistortionHalfResEnable;
    bool Distortion8BitEnable;
    bool DistortionTilingEnable;
    bool StaticEnvmapEnable;
    bool CustomEnvmapEnable;
    bool CustomEnvmapMipmapClampEnable;
    bool SkyEnvmapEnable;
    bool SkyEnvmapMipmapGenEnable;
    bool SkyEnvmapUpdateEnable;
    bool SkyEnvmapForceUpdateEnable;
    bool SkyEnvmapUseFastHDR;
    bool SkyEnvmapUse32bitLatLongTexture;
    bool SkyEnvmapDebugColorEnable;
    bool DynamicEnvmapEnable;
    bool DynamicEnvmapMipmapGenEnable;
    bool DrawDebugDynamicEnvmap;
    bool DynamicEnvmapShadowmapEnable;
    bool DynamicEnvmapShadowmapFarPlaneOverride;
    bool DynamicEnvmapShadowmapShadowExtrusionOverride;
    bool DrawDebugDynamicEnvmapShadowmap;
    bool DrawDynamicEnvmapFrustums;
    bool HairCoverageEnable;
    bool SetupJobEnable;
    bool FinishSyncJobsFirstEnable;
    bool PrepareDispatchListJobEnable;
};
struct WorldRenderSettings : WorldRenderSettingsBase {
    uint32_t GenericEntityMaxVisibleEntityCount;
    uint32_t DrawDebugGroundHeight;
    float DecalVolumeScale;
    uint32_t MaxLensFlaresPerFrame;
    QualityLevel LensFlaresQualityLevel;
    uint32_t GBufferLayout;
    uint32_t GBufferTestCount;
    float GBufferAlphaTestSimpleSmoothness;
    float GBufferForceSmoothness;
    float GBufferForceSpecularAlbedo;
    int32_t HierarchicalZJitterForceIndex;
    uint32_t OutdoorLightTileBatchCount;
    int32_t OnlyLightTileIndex;
    uint32_t EmitterSunTransmittanceMapResolution;
    uint32_t MaxDestructionVolumeCount;
    uint32_t MaxDecalVolumeCount;
    uint32_t LightTileCsAvgLightCountPerTile;
    float LightCullFrustumExpandDistance;
    LightTileDebugLightCountMode LightTileDebugLightMode;
    int32_t LightTileDebugColorMode;
    uint32_t DebugLightStatsLightCountHighwatermark;
    float LightLodFadeArea;
    float LightLodMinArea;
    float LightLodRadiusFactor;
    uint32_t OcclusionCullingWidth;
    uint32_t OcclusionCullingHeight;
    uint32_t OcclusionTriangleCount;
    uint32_t ShadowOcclusionCullingWidth;
    uint32_t ShadowOcclusionCullingHeight;
    uint32_t ShadowOcclusionTriangleCount;
    float FrustumSilhouetteCullingPadding;
    int32_t SubSurfaceScatteringSampleCount;
    float SubsurfaceBlurPixelRadiusCullThreshold;
    int32_t OnlyTileIndex;
    float PlanarReflectionViewScale;
    float PlanarReflectionConvolutionSampleClampThreshold;
    uint32_t PlanarReflectionConvolutionSampleCount;
    float PlanarReflectionCullFOV;
    float PlanarReflectionFarPlane;
    uint32_t DrawDebugPlanarReflectionMipLevel;
    uint32_t DrawDebugPlanarReflectionMode;
    float ReflectionLodScale;
    PostProcessAAMode PostProcessAntialiasingMode;
    uint32_t TemporalAAJitterCount;
    float TemporalAASharpness;
    float TemporalAAMinHistoryBlendFactor;
    float TemporalAAMaxHistoryBlendFactor;
    float TemporalAADisocclusionRejectionFactor;
    float TemporalAALumaContrastFactor;
    float TemporalAAMotionSharpeningFactor;
    float TemporalAAAntiflickerMultiplier;
    float TemporalAAAntiflickerInDistance;
    float TemporalAAAntiflickerOutDistance;
    uint32_t DrawDebugTemporalAAAccumulationCount;
    uint32_t DrawDebugTemporalAADebugMode;
    float DrawDebugTemporalAAMaxDistance;
    float TemporalAAResponsiveness;
    float TemporalAAAntiflickerStrength;
    uint32_t TemporalAAQuality;
    ScaleResampleMode RenderScaleResampleMode;
    QualityLevel SkyCelestialQuality;
    QualityScalableInt SkyCelestialMaxQuadCount;
    SkyRenderMode SkyRenderMode;
    float InterpupillaryDistance;
    uint32_t SpotLightShadowmapResolution;
    uint32_t SpotLightShadowmapQuality;
    float SpotLightShadowmapPoissonFilterScale;
    SpotLightShadowmapTextureMode SpotLightShadowmapTextureMode;
    float SpotLightShadowMaxAngle;
    float SpotLightShadowFadeOutRange;
    uint32_t MaxShadowCount;
    uint32_t MaxPunctualLightCount;
    uint32_t MaxPunctualShadowLightCount;
    uint32_t MaxAreaLightCount;
    uint32_t MaxAreaShadowLightCount;
    uint32_t MaxLocalReflectionVolumeCount;
    uint32_t MaxLocalPlanarReflectionCount;
    uint32_t MaxPunctualRectangularLightCount;
    QualityLevel PunctualLightShadowLevel;
    QualityLevel AreaLightShadowLevel;
    uint32_t LocalIBLMaxFaceCapture;
    uint32_t LocalIBLLightingUpdateCount;
    uint32_t LocalIBLRefreshDelayCount;
    float LocalIBLSunUpdateThreshold;
    uint32_t LocalIBLShadowmapSliceCount;
    uint32_t LocalIBLShadowmapResolution;
    uint32_t PBRLocalIBLAcquisitionWaitFrameCount;
    uint32_t PBRDiffuseConvolutionMipLevelOffset;
    uint32_t PBRSpecularConvolutionSampleCount;
    uint32_t PBRDebugSpecularConvolutionSampleCount;
    uint32_t LocalIBLResolution;
    float DrawDebugLocalIBLPreviewScale;
    uint32_t DrawDebugLocalIBLIndex;
    uint32_t DrawDebugLocalIBLMipLevel;
    uint32_t ContinuousLocalIBLIndex;
    uint32_t MaxLocalPlanarReflectionTargetCount;
    float PBRMaxIlluminanceValue;
    float DiffuseRangeSRGBMinLimitValue;
    float DiffuseRangeSRGBMinValue;
    float DiffuseRangeSRGBMaxValue;
    float DiffuseRangeSRGBMaxLimitValue;
    float VolumetricLightCascadeBaseVoxelSize;
    float VolumetricLightCascadeVoxelSizeCascadeFactor;
    uint32_t VolumetricLightCascadeResolution;
    float VolumetricDensityCascadeBaseVoxelSize;
    float VolumetricDensityCascadeVoxelSizeCascadeFactor;
    uint32_t VolumetricDensityCascadeResolution;
    float VolumetricLightingIncreaseTemporalSmoothingFactor;
    float VolumetricLightingDecreaseTemporalSmoothingFactor;
    uint32_t VolumetricShadowQuality;
    uint32_t VolumetricShadowmapResolution;
    uint32_t VolumetricShadowmapMaxCount;
    QualityLevel PunctualLightCastVolumetricShadowmapEnableLevel;
    QualityLevel AreaLightCastVolumetricShadowmapEnableLevel;
    uint32_t VolumetricParticlesInjectionMode;
    uint32_t DrawDebugVolumetricDensity;
    uint32_t DrawDebugVolumetricLight;
    float DrawGpuHistogramHDRMinEV;
    float DrawGpuHistogramHDRMaxEV;
    uint32_t DrawGpuHistogramBinCount;
    uint32_t NumberOfEntitiesPerPartition;
    int32_t ForceTemporalAAQuality;
    bool TestRenderingEnable;
    bool GenericEntityRendererEnable;
    bool ZBufferShadowTestEnable;
    bool DecalVolumeEnable;
    bool DrawDebugDecalVolumes;
    bool DrawDebugDestructionVolumes;
    bool LensFlaresEnable;
    bool DrawDebugLensFlareOccluders;
    bool DrawDebugLensFlares;
    bool LensFlareOcclusionEnable;
    bool CloudShadowEnable;
    bool OverrideDynamicAO;
    bool DrawDebugDynamicAO;
    bool FilmicEffectsEnable;
    bool EmissiveEnable;
    bool GBufferClearEnable;
    bool DxGBufferLight16BitEnable;
    bool DxGBufferNormal16BitEnable;
    bool DxGBufferRoughness16BitEnable;
    bool GBufferAlphaTestSimpleEnable;
    bool Gen4aEsramEnable;
    bool Gen4aHierarchicalZEsramEnable;
    bool Gen4aScreenSpaceRaytraceEsramEnable;
    bool HierarchicalZJitterEnable;
    bool SpecularLightingEnable;
    bool SkinLightingEnable;
    bool TranslucencyLightingEnable;
    bool TranslucencyAutoThicknessEnable;
    bool DynamicEnvmapLightingEnable;
    bool OutdoorLightEnable;
    bool LightStencilMethodEnable;
    bool LightVolumeMethodEnable;
    bool LightVolumeDepthTestEnable;
    bool OutdoorKeyLightEnable;
    bool OutdoorSkyLightEnable;
    bool OutdoorLightTilingEnable;
    bool OutdoorLightTileRenderEnable;
    bool OutdoorLightTileBlendEnable;
    bool OutdoorLightTileSimpleShaderEnable;
    bool EmitterSunTransmittanceMapEnable;
    bool RadiositySpotLightShadowCullingEnable;
    bool LightTileCombineOutdoorLightEnable;
    bool LightTileCsPathEnable;
    bool LightTileAsyncComputeCulling;
    bool LightTileMinMaxUseHTile;
    bool LightTileUseCullingHierarchy;
    bool LightTileUseDetailedGpuTimers;
    bool LightTileUseCsIndirectClears;
    bool DrawDebugLightStats;
    bool DrawDebugLightStatsForward;
    bool DrawDebugLightSources;
    bool DrawDebugLightShadowSources;
    bool DrawDebugLightShadowStats;
    bool DrawDebugGBuffer;
    bool DrawDebugMaterialInput;
    bool DrawDebugMaterialOutput;
    bool DrawDebugLightEmissiveSurface;
    bool UseNewLightCullEnable;
    bool LightCullEnable;
    bool LightOcclusionCullEnable;
    bool LightConeCullEnable;
    bool LocalIBLOcclusionCullingEnable;
    bool ShadowOcclusionCullingEnable;
    bool FrustumSilhouetteCullingEnable;
    bool SubSurfaceScatteringEnable;
    bool TranslucencyEnable;
    bool SplitLightingEnable;
    bool SubsurfaceBlurComputeEnable;
    bool SubsurfaceBlurQuadtreeTileGenEnable;
    bool OpaqueSortBySolutionEnable;
    bool MainOpaqueZPassEnable;
    bool PlanarReflectionEnable;
    bool PlanarReflectionFastHdrEnable;
    bool PlanarReflectionBlurEnable;
    bool PlanarReflectionConvolutionEnable;
    bool PlanarReflectionConvolutionRandomSamplesEnable;
    bool PlanarReflectionConvolutionPostFilterEnable;
    bool PlanarReflectionClippingEnable;
    bool DrawDebugPlanarReflection;
    bool DrawDebugPlanarReflectionCullFrustum;
    bool LocalPlanarReflectionConvolutionEnable;
    bool OverlayEnable;
    bool OverlayZTestEnable;
    bool OutlineEnable;
    bool SmaaVelocityReprojectionEnable;
    bool SmaaUseStencil;
    bool SmaaPredicatedThresholdingEnable;
    bool TemporalAAJitterUseCmj;
    bool TemporalAASmoothHistoryFiltering;
    bool TemporalAAAsyncCompute;
    bool DrawDebugTemporalAAEnable;
    bool TemporalAADofCocFilterEnable;
    bool TemporalAAHistorySharpeningEnable;
    bool RenderScaleResampleEnable;
    bool SkyCelestialEnable;
    bool FullscreenLensReflectionEnable;
    bool SpriteDOFBeforeMotionBlur;
    bool VrHmdLensDistortionEnable;
    bool VrHmdLateReprojectionEnable;
    bool SpotLightShadowmapEnable;
    bool PBRSupportOriginalLight;
    bool RadiosityShadowCullingEnable;
    bool PunctualLightsEnable;
    bool AreaLightsEnable;
    bool LocalReflectionEnable;
    bool TilePassPunctualLightsEnable;
    bool TilePassPunctualLightShadowEnable;
    bool TilePassAreaLightsEnable;
    bool TilePassAreaLightShadowEnable;
    bool TilePassLocalReflectionVolumeEnable;
    bool TilePassLocalPlanarReflectionEnable;
    bool SphereLightsEnable;
    bool PunctualSphereLightsEnable;
    bool SpotLightsEnable;
    bool PunctualSpotLightsEnable;
    bool TubeLightsEnable;
    bool PunctualTubeLightsEnable;
    bool RectangularLightsEnable;
    bool PunctualRectangularLightsEnable;
    bool LocalReflectionVolumeSphereEnable;
    bool LocalReflectionVolumeBoxEnable;
    bool LocalPlanarReflectionEnable;
    bool LocalIBLUpdateWithSkyEnable;
    bool LocalIBLUpdateWithEnlightenSkyBoxChange;
    bool LocalIBLSunSpecularOcclusionEnable;
    bool LocalIBLBoxCullingEnable;
    bool LocalIBLShadowmapFaceMerging;
    bool PBRLocalIBLFogEnable;
    bool PBRDrawDiffuseReference;
    bool PBRDrawSpecularReference;
    bool PBRDrawLocalIBLReference;
    bool PBRDrawDistantIBLReference;
    bool PBRDrawAreaLightReference;
    bool PBRConvolutionMISEnable;
    bool PBRConvolutionHighestMIPEnable;
    bool PBRConvolutionCubeArrayEnable;
    bool PBRConvolutionChainEnable;
    bool DrawDebugLocalIBLEnable;
    bool DrawDebugLocalIBLStatsEnable;
    bool DrawDebugLocalIBLShadowmaps;
    bool DrawDebugPreIntegratedFGTexture;
    bool DrawDebugReflectionState;
    bool DrawDebugProbeMirrorEnable;
    bool DrawDebugProbeDiffuseEnable;
    bool DrawDebugProbeScreenEnable;
    bool DrawDebugProbeScreenOnRight;
    bool ContinuousLocalIBLEnable;
    bool PBRConvolutionPrecomputedSampleEnable;
    bool PBRConvolutionComputeEnable;
    bool PBRConvolutionRandomRotationEnable;
    bool DrawDebugLocalPlanarReflections;
    bool EmitterQuadRenderingEnable;
    bool EmitterMeshRenderingEnable;
    bool EmitterPointLightsEnable;
    bool EmitterSpotLightsEnable;
    bool UseSSSProfileforOATS;
    bool DeterministicRenderingEnable;
    bool HdrNanInfRemovalEnable;
    bool HdrNanInfRemovalForceEnable;
    bool VolumetricRenderingEnable;
    bool VolumetricCascadePositionUpdateEnable;
    bool VolumetricLightingTemporalAAEnable;
    bool VolumetricLightingUpsamplePreviousCascade;
    bool VolumetricShadowSkipLowerMipSamples;
    bool VolumetricShadowCascadeBasedQuality;
    bool VolumetricShadowmapEnable;
    bool VolumetricParticlesDensityInjectionEnable;
    bool EmitterVolumetricLightingEnable;
    bool DrawDebugVolumetricCascadedVolumesEnable;
    bool DrawDebugVolumetricShadowMaps;
    bool DrawDebugVolumetricEmitterInjectingDensityEnable;
    bool LightShaftFastHdrEnable;
    bool DrawGpuHistogramEnable;
    bool DrawGpuHistogramRed;
    bool DrawGpuHistogramBlue;
    bool DrawGpuHistogramGreen;
    bool DrawGpuHistogramLuminance;
    bool DrawGpuHistogramHDRMode;
    bool EntityRendererPartitioningEnable;
    bool DrawDebugEntityRendererPartitions;
    bool VehicleEntityForegroundZPassEnable;
    bool SoldierRenderFirstPersonTransformEnable;
    bool SelectiveStaticModelEntityZPassEnable;
    bool AfterTAATransparentEnable;
    bool AfterTAAForegroundNoDepthEnable;
    bool VehicleEntityEnableMeshComponentCulling;
    bool ForceTemporalAAOff;

    static WorldRenderSettings* GetInstance() {
        return *reinterpret_cast<WorldRenderSettings**>(OFFSETWORLDRENDERSETTINGS);
    }
};


struct GlobalPostProcessSettings {
    char padding[4];                //PostProcessDebugMode DebugMode;
    uint32_t DebugModeStep;
    Vec3 ForceBloomScale;
    Vec4 ForceVignetteColor;
    Vec3 FilmGrainColorScale;
    Vec3 Brightness;
    Vec3 Contrast;
    Vec3 Saturation;
    float ForceEVCompensation;
    float ForceEV;
    int32_t BloomFFTMipLevel;
    int32_t BloomFFTProceduralKernelSize;
    uint32_t BlurPyramidWidth;
    uint32_t BlurPyramidHeight;
    uint32_t BlurPyramidFinalLevel;
    float BlurPyramidLdrRange;
    float DebugColorGraphMinValue;
    float DebugColorGraphMaxValue;
    int32_t DebugColorGraphLineNumber;
    char padding2[4];               //AutoExposureMethod AutoExposureMethod;
    uint32_t AutoExposureHistogramBinCount;
    uint32_t AutoExposureHistogramMipUsed;
    float AutoExposureHistogramMinValue;
    float AutoExposureHistogramMaxValue;
    uint32_t DownsampleAverageStartMipmap;
    int32_t ForceDofEnable;
    float ForceDofBlurFactor;
    float ForceDofBlurAdd;
    float ForceDofFocusDistance;
    float ForceSimpleDofNearStart;
    float ForceSimpleDofNearEnd;
    float ForceSimpleDofFarStart;
    float ForceSimpleDofFarEnd;
    float ForceSimpleDofBlurMax;
    float ForceSpriteDofNearStart;
    float ForceSpriteDofNearEnd;
    float ForceSpriteDofFarStart;
    float ForceSpriteDofFarEnd;
    float ForceSpriteDofBlurMax;
    Vec2 ForceVignetteScale;
    float ForceVignetteExponent;
    float FxaaComputeSubPixelRemoval;
    float FxaaComputeContrastThreshold;
    int32_t ForceTonemapMethod;
    char padding3[4];               //ColorGradingQualityMode ColorGradingHighQualityMode;
    int32_t ForceChromostereopsisEnable;
    int32_t ForceChromostereopsisOffset;
    float ForceChromostereopsisScale;
    Vec2 FilmGrainTextureScale;
    float LensScopeColorScale;
    float HalfResEdgeDetectThreshold;
    float Hue;
    float UIBrightnessNorm;
    float UserBrightnessMin;
    float UserBrightnessMax;
    float UserBrightnessAddScale;
    float UserBrightnessMulScale;
    float LUTGammaR;
    float LUTGammaG;
    float LUTGammaB;
    float LUTGammaCurbOffset;
    char padding4[4];               //DofMethod DofMethod;
    char padding5[4];               //BlurMethod BlurMethod;
    float SpriteDofMinRadiusLayer1;
    float SpriteDofMinRadiusLayer2;
    float SpriteDofMaxRadiusGatherPass;
    float SpriteDofMergeColorThreshold;
    float SpriteDofMergeRadiusThreshold;
    float SpriteDofDepthDiscontinuityThreshold;
    uint32_t SpriteDofActiveLayer;
    float SpriteDofInfocusMultiplier;
    float SpriteDofMaxBlurScale;
    float SpriteDofEnergyScaler;
    uint32_t SpriteDofMultilayerForegroundCount;
    float SpriteDofMultilayerForegroundCocSpan;
    float SpriteDofForegroundReweightExponent;
    float SpriteDofMultilayerForegroundLayerExtension;
    uint32_t SpriteDofMultilayerForegroundActiveLayer;
    char padding6[4];               //DynamicAOMethod DynamicAOMethod;
    float ScreenSpaceRaytraceThicknessFadeScale;
    float ScreenSpaceRaytraceReuseAllocWhitePoint;
    int32_t ScreenSpaceRaytraceDebug;
    int32_t ScreenSpaceRaytraceQuality;
    uint32_t IronsightsDofResolutionFactor;
    char padding7[4];
    //BlurFilter IronsightsBlurFilter;
    //BlurFilter IronsightsBlurFilter720p;
    float IronsightsHDRCompression;
    float IronsightsCoCScale;
    float OverrideIronsightsHipFade;
    float OverrideIronsightsStartFade;
    float OverrideIronsightsFocalDistance;
    float OverrideIronsightsDofCircleDistance;
    float SetAlphaFromDepthBufferThreshold;
    uint32_t DynamicAOSampleTemporalCount;
    uint32_t DynamicAOSampleStepCount;
    uint32_t DynamicAOSampleDirCount;
    float DynamicAOMaxFootprintRadius;
    uint32_t DynamicAOBilateralBlurRadius;
    float DynamicAOBilateralBlurSharpness;
    float DynamicAONormalInfluence;
    float DynamicAOTemporalSharpness;
    float DynamicAOTemporalMinHistoryBlendFactor;
    float DynamicAOTemporalMaxHistoryBlendFactor;
    float DynamicAOTemporalDisocclusionRejectionFactor;
    float DynamicAOTemporalLumaContrastFactor;
    float DynamicAOTemporalMotionSharpeningFactor;
    float DynamicAOTemporalAntiflickerMultiplier;
    float DynamicAOTemporalAntiflickerInDistance;
    float DynamicAOTemporalAntiflickerOutDistance;
    uint32_t DrawDebugDynamicAOTemporalAccumulationCount;
    uint32_t DrawDebugDynamicAOTemporalDebugMode;
    float DrawDebugDynamicAOTemporalMaxDistance;
    float TemporalAAUnclampStillPixelsThreshold;
    bool HdrBlurEnable;
    bool EVClampEnable;
    bool AdaptationTimeEnable;
    bool ForceEVCompensationEnable;
    bool ForceEVEnable;
    bool DrawDebugInfo;
    bool DrawExposureDebugInfo;
    bool RenderTargetLoadOptsEnable;
    bool BlurEnable;
    bool QuarterDownsamplingEnable;
    bool BlurBlendEnable;
    bool BloomEnable;
    bool BloomTestEnable;
    bool BloomFFTEnable;
    bool DrawDebugFFTEnable;
    bool BlurPyramidEnable;
    bool BlurPyramidSizeOverrideEnable;
    bool BlurPyramidQuarterResEnable;
    bool BlurPyramidHdrEnable;
    bool BlurPyramidFastHdrEnable;
    bool BlurPyramidSinglePassEnable;
    bool DebugColorGraphEnable;
    bool DownsampleLogAverageEnable;
    bool DownsampleBeforeBlurEnable;
    bool VignetteEnable;
    bool FxaaComputeDebug;
    bool ColorGradingEnable;
    bool ColorGradingComputeEnable;
    bool ColorGradingDebugEnable;
    bool ColorTransformEnable;
    bool ColorGradingForceUpdateAlways;
    bool FilmGrainEnable;
    bool FilmGrainLinearFilteringEnable;
    bool FilmGrainRandomEnable;
    bool LensScopeEnable;
    bool UserBrightnessLUTEnable;
    bool DrawDebugUserBrightnessLUT;
    bool SpriteDofEnable;
    bool SpriteDofMergeEnable;
    bool SpriteDofForegroundEnable;
    bool SpriteDofDepthFilterEnable;
    bool SpriteDofBuffer32bitEnable;
    bool SpriteDofHalfResolutionEnable;
    bool SpriteDofInstancingEnable;
    bool SpriteDofNearGatherEnable;
    bool SpriteDofMultilayerForegroundEnable;
    bool SpriteDofDebugEnable;
    bool DynamicAOEnable;
    bool SsaoBlurEnable;
    bool ScreenSpaceRaytraceEnable;
    bool ScreenSpaceRaytraceDeferredResolveEnable;
    bool ScreenSpaceRaytraceSeparateCoverageEnable;
    bool ScreenSpaceRaytraceFullresEnable;
    bool ScreenSpaceRaytrace32BitEnable;
    bool ScreenSpaceRaytraceReuseAllocEnable;
    bool IronsightsDofEnable;
    bool ForceIronsightsDofActive;
    bool OverrideIronsightsDofParams;
    bool OverrideIronsightsDofCircleBlur;
    bool ForceLensScopeActive;
    bool DynamicAOHorizonBased;
    bool DynamicAOBilateralBlurEnable;
    bool DynamicAONormalEnable;
    bool DynamicAOUseAsyncCompute;
    bool DynamicAOTemporalFilterEnable;
    bool DynamicAOTemporalSmoothHistoryFiltering;
    bool DrawDebugDynamicAOTemporalEnable;
    bool TemporalAAUseImprovedVersion;
};

struct UISettings {

    char padding[4];            //UISystemType System;
    char padding2[8];           //PointerRef<ProfileOptionsAsset> ProfileOptions;
    char padding3[4];           //LanguageFormat Language;
    char padding4[16];
    //PointerRef<LocalizationAsset> Localization;
    //PointerRef<UIImmediateModeFontConfigurationAsset> FontConfiguration;
    bool DrawEnable;
    bool EnableJobs;
    bool UseXYUITranslationOffset;
    bool DeferScreenCreation;

    //@Todo, use SystemSettings to not do this like Kyber does
    static UISettings* GetInstance() {
        return *reinterpret_cast<UISettings**>(OFFSETUISETTINGS);
    }
};

///////////////////////////////////
// Game
///////////////////////////////////

class GamepadState {
 public:
    DWORD bitmask;  // 0x0000
    char pad_0004[12];  // 0x0004
    float U;  // 0x0010
    float D;  // 0x0014
    float L;  // 0x0018
    float R;  // 0x001C
    float Y;  // 0x0020
    float A;  // 0x0024
    float X;  // 0x0028
    float B;  // 0x002C
    char pad_0030[8];  // 0x0030
    float L3;  // 0x0038
    float R3;  // 0x003C
    float Menu;  // 0x0040
    float Start;  // 0x0044
    float LT;  // 0x0048
    float RT;  // 0x004C
    float LB;  // 0x0050
    float RB;  // 0x0054
    char pad_0058[184];  // 0x0058
    Vec2 LeftThumb;  // 0x0110
    Vec2 RightThumb;  // 0x0118
    char pad_0120[64];  // 0x0120
};  // Size: 0x0160
enum LogFileCollisionMode
{
    LFCM_Overwrite, // 0x0000
    LFCM_Rotate,    // 0x0001
    LFCM_TimeStamp  // 0x0002
};
class DataContainer
{
public:
    char _0x000[24]; // 0x0000
};
class Asset : public DataContainer
{
public:
    char* Name; // 0x0018
};
class PlayerData : public Asset
{
public:
    //PlayerViewData* PlayerView; // 0x0020
    //EntryInputActionMapsData* InputConceptDefinition;
    //InputActionMappingsData* InputMapping;
};
class GameModeViewDefinition : public Asset
{
public:
    char* GameModeName;                    // 0x0020
    char padding[8];                 //PlayerViewDefinition* ViewDefinitions; // 0x0028
    float MaxVariableFps;
};
class GameSettings
{
public:
    uint32_t MaxPlayerCount;
    uint32_t MaxSpectatorCount;
    uint32_t MinPlayerCountElimination;
    LogFileCollisionMode LogFileCollisionMode;
    uint32_t LogFileRotationHistoryLength;
    char* Level;
    char* StartPoint;
    char* InstallationLevel;
    char* InstallationStartPoint;
    char* InstallationDefaultLayerInclusion;
    char _0x000[8];                          // InputConfiguration InputConfiguration;
    char* ActiveGameModeViewDefinition;
    GameModeViewDefinition** GameModeViewDefinitions;
    char padding[4]; //TeamId DefaultTeamId;
    uint32_t PS3ContentRatingAge;
    uint32_t LogHistory;
    char padding2[16];      
    //VersionData* Version;
    //SubWorldInclusion* LayerInclusionTable;
    char* DefaultLayerInclusion;
    float TimeBeforeSpawnIsAllowed;
    float LevelWarmUpTime;
    float TimeToWaitForQuitTaskCompletion;
    PlayerData* Player;
    char pad_test[8]; // DifficultyDatas DifficultySettings;
    uint32_t DifficultyIndex;
    char padding3[8];//SKU CurrentSKU;
    //GameSettingsComponent GameSettingsComponents;
    bool LogFileEnable;
    bool ResourceRefreshAlwaysAllowed;
    bool SpawnMaxLocalPlayersOnStartup;
    bool UseSpeedBasedDetailedCollision;
    bool UseSingleWeaponSelector;
    bool AutoAimEnabled;
    bool HasUnlimitedAmmo;
    bool HasUnlimitedMags;
    bool RotateLogs;
    bool AdjustVehicleCenterOfMass;
    bool AimAssistEnabled;
    bool AimAssistUsePolynomials;
    bool ForceFreeStreaming;
    bool ForceDisableFreeStreaming;
    bool IsGodMode;
    bool IsJesusMode;
    bool IsJesusModeAi;
    bool GameAdministrationEnabled;
    bool AllowDestructionOutsideCombatArea;
    bool DefaultCameraInheritsFov;
};  // Size: 0x0080

class GameContext {
 public:
    char pad_0000[56];  // 0x0000
    class GameSettings* gameSettings;  // 0x0038
    char pad_0040[24];  // 0x0040
    class ClientPlayerManager* clientPlayerManager;  // 0x0058

    static GameContext* GetInstance() {
        return *reinterpret_cast<GameContext**>(OFFSETGAMECONTEXT);
    }
};  // Size: 0x0060

class ClientPlayerManager
{
public:
    char pad_0000[8];
    class PlayerData* m_playerData;
    uint32_t m_maxPlayerCount;      // 0x0010
    uint32_t m_playerCountBitCount; // 0x0014
    uint32_t m_playerIdBitCount;    // 0x0018
    char pad_001C[224];
    // Technically the players, spectators, have their own structure but they're similar enough that I can use the ServerPlayer
    ClientPlayer* m_players[64]; // 0x0100 
    ClientPlayer* m_spectators[64];
    ClientPlayer* m_localPlayers[64];
};  // Size: 0x0570

class ClientPlayer {
 public:
    void* vtable;                     // 0x0000
    class PlayerData* m_data;         // 0x0008
    class MemoryArena* m_memoryArena; // 0x0010
    char* m_name;                     // 0x0018
    char pad_0020[20];                // 0x0020

    //@Todo fix these offsets
    class ClientControllableEntity* attachedControllable;  // 0x0200
    char pad_0208[8];  // 0x0208
    class ClientControllableEntity* controlledcontrollable;  // 0x0210
    // Swbf 2015 Team Id is like 0x25F8 or something crazy. From player extent bs I don't wanna think about
};  // Size: 0x0218

class ClientControllableEntity {
 public:
    char pad_0000[1236];  // 0x0000
    float HeightOffset;  // 0x04D4
    char pad_04D8[640];  // 0x04D8
    class ClientSoldierPrediction* clientSoldierPrediction;
};  // Size: 0x0760

class ClientSoldierPrediction {
 public:
    char pad_0000[32];  // 0x0000
    Vector3 location;  // 0x0020
    char pad_002C[8];  // 0x002C
    Vector3 velocity;  // 0x0034
};  // Size: 0x0040

class LocalAimer {
 public:
    char pad_0000[152];  // 0x0000
    class AimingComponentSwitch* aimingComponentSwitch;  // 0x0098

    static LocalAimer* GetInstance() {
        return *reinterpret_cast<LocalAimer**>(OFFSETLOCALAIMER);
    }
};  // Size: 0x00A0

class AimingComponentSwitch {
 public:
    char pad_0000[56];  // 0x0000
    class AimingComponentData* primary;  // 0x0038
    char pad_0040[104];  // 0x0040
    class AimingComponentData* secondary;  // 0x00A8
};  // Size: 0x00B0

class AimingComponentData {
 public:
    unsigned char signature[12];  // 0x0000
    char pad_000C[156];  // 0x000C
    float yaw;  // 0x00A8
    float pitch;  // 0x00AC
};  // Size: 0x00B4
