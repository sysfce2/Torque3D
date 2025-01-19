#include "../../postFX/postFx.hlsl"
#include "../../shaderModel.hlsl"
#include "../../shaderModelAutoGen.hlsl"
#include "../../lighting.hlsl"
#include "../../brdf.hlsl"

TORQUE_UNIFORM_SAMPLER2D(deferredBuffer, 0);
TORQUE_UNIFORM_SAMPLER2D(colorBuffer, 1);
TORQUE_UNIFORM_SAMPLER2D(matInfoBuffer, 2);
TORQUE_UNIFORM_SAMPLER2D(BRDFTexture, 3);
uniform float3 ambientColor;
uniform float4 rtParams0;
uniform float4 vsFarPlane;
uniform float4x4 cameraToWorld;

//cubemap arrays require all the same size. so shared mips# value
uniform float cubeMips;

uniform int numProbes;

TORQUE_UNIFORM_SAMPLERCUBEARRAY(specularCubemapAR, 4);
TORQUE_UNIFORM_SAMPLERCUBEARRAY(irradianceCubemapAR, 5);
TORQUE_UNIFORM_SAMPLER2D(WetnessTexture, 6);

#ifdef USE_SSAO_MASK
TORQUE_UNIFORM_SAMPLER2D(ssaoMask, 7);
uniform float4 rtParams7;
#endif
uniform float accumTime;
uniform float dampness;

uniform float4    probePosArray[MAX_PROBES];
uniform float4    refPosArray[MAX_PROBES];
uniform float4x4  worldToObjArray[MAX_PROBES];
uniform float4    refScaleArray[MAX_PROBES];
uniform float4    probeConfigData[MAX_PROBES];   //r,g,b/mode,radius,atten

#if DEBUGVIZ_CONTRIB
uniform float4    probeContribColors[MAX_PROBES];
#endif

uniform int skylightCubemapIdx;
uniform int SkylightDamp;

float4 main(PFXVertToPix IN) : SV_TARGET
{
   //unpack normal and linear depth 
   float4 normDepth = TORQUE_DEFERRED_UNCONDITION(deferredBuffer, IN.uv0.xy);

   //create surface
   Surface surface = createSurface(normDepth, TORQUE_SAMPLER2D_MAKEARG(colorBuffer),TORQUE_SAMPLER2D_MAKEARG(matInfoBuffer),
      IN.uv0.xy, eyePosWorld, IN.wsEyeRay, cameraToWorld);
      
   if (getFlag(surface.matFlag, 2))
   {
      return surface.baseColor;
   } 
   
   #ifdef USE_SSAO_MASK
      float ssao =  1.0 - TORQUE_TEX2D( ssaoMask, viewportCoordToRenderTarget( IN.uv0.xy, rtParams7 ) ).r;
      surface.ao = min(surface.ao, ssao);  
   #endif

   float alpha = 1;
   float wetAmmout = 0;
#if SKYLIGHT_ONLY == 0
   int i = 0;
   float blendFactor[MAX_PROBES];
   float blendSum = 0;
   float blendFacSum = 0;
   float invBlendSum = 0;
   float probehits = 0;
   //Set up our struct data
   float contribution[MAX_PROBES];
   
   float blendCap = 0;
   if (alpha > 0)
   {
      //Process prooooobes
      for (i = 0; i < numProbes; i++)
      {
         contribution[i] = 0.0;

         float atten =1.0-(length(eyePosWorld-probePosArray[i].xyz)/maxProbeDrawDistance);
         if (probeConfigData[i].r == 0) //box
         { 
            contribution[i] = defineBoxSpaceInfluence(surface.P, worldToObjArray[i], probeConfigData[i].b)*atten;
         }
         else if (probeConfigData[i].r == 1) //sphere  
         { 
            contribution[i] = defineSphereSpaceInfluence(surface.P, probePosArray[i].xyz, probeConfigData[i].g, probeConfigData[i].b)*atten; 
         }

            if (contribution[i]>0.0)
               probehits++;
         else
            contribution[i] = 0.0;

         if (refScaleArray[i].w>0)
            wetAmmout += contribution[i];
         else
            wetAmmout -= contribution[i];

         blendSum += contribution[i];
         blendCap = max(contribution[i],blendCap);
      }
      if (wetAmmout<0) wetAmmout =0;
       if (probehits > 0.0)
	   {
         invBlendSum = (probehits - blendSum)/probehits; //grab the remainder 
         for (i = 0; i < numProbes; i++)
         {
               blendFactor[i] = contribution[i]/blendSum; //what % total is this instance
               blendFactor[i] *= blendFactor[i]/invBlendSum;  //what should we add to sum to 1
               blendFacSum += blendFactor[i]; //running tally of results
         }

         for (i = 0; i < numProbes; i++)
         {
            //normalize, but in the range of the highest value applied
            //to preserve blend vs skylight
            contribution[i] = blendFactor[i]/blendFacSum*blendCap;
         }
      }
      
#if DEBUGVIZ_ATTENUATION == 1
      float contribAlpha = 0;
      for (i = 0; i < numProbes; i++)
      {
         contribAlpha += contribution[i];
      }

      return float4(contribAlpha,contribAlpha,contribAlpha, 1);
#endif

#if DEBUGVIZ_CONTRIB == 1
      float3 finalContribColor = float3(0, 0, 0);
      for (i = 0; i < numProbes; i++)
      {
         finalContribColor += contribution[i] * float3(fmod(i+1,2),fmod(i+1,3),fmod(i+1,4));
      }
      return float4(finalContribColor, 1);
#endif
   }
   for (i = 0; i < numProbes; i++)
   {
      float contrib = contribution[i];
      if (contrib > 0.0f)
      {
         alpha -= contrib;
      }
   }
#endif

   float3 irradiance = float3(0, 0, 0);
   float3 specular = float3(0, 0, 0);

   if (SkylightDamp>0)
      wetAmmout += alpha;
   dampen(surface, TORQUE_SAMPLER2D_MAKEARG(WetnessTexture), accumTime, wetAmmout*dampness);
   
   // Radiance (Specular)
#if DEBUGVIZ_SPECCUBEMAP == 0
   float lod = roughnessToMipLevel(surface.roughness, cubeMips);
#elif DEBUGVIZ_SPECCUBEMAP == 1
   float lod = 0;
#endif

#if SKYLIGHT_ONLY == 0
   for (i = 0; i < numProbes; i++)
   {
      float contrib = contribution[i];
      if (contrib > 0.0f)
      {
         int cubemapIdx = probeConfigData[i].a;
         float3 dir = boxProject(surface.P-refPosArray[i].xyz, surface.R, worldToObjArray[i], refScaleArray[i].xyz, probePosArray[i].xyz);

         irradiance += TORQUE_TEXCUBEARRAYLOD(irradianceCubemapAR, dir, cubemapIdx, 0).xyz * contrib;
         specular += TORQUE_TEXCUBEARRAYLOD(specularCubemapAR, dir, cubemapIdx, lod).xyz * contrib;
      }
   }
#endif
   if(skylightCubemapIdx != -1 && alpha >= 0.001)
   {
      irradiance = lerp(irradiance,TORQUE_TEXCUBEARRAYLOD(irradianceCubemapAR, surface.R, skylightCubemapIdx, 0).xyz,alpha);
      specular = lerp(specular,TORQUE_TEXCUBEARRAYLOD(specularCubemapAR, surface.R, skylightCubemapIdx, lod).xyz,alpha);
   }

#if DEBUGVIZ_SPECCUBEMAP == 1 && DEBUGVIZ_DIFFCUBEMAP == 0
   return float4(specular, 1);
#elif DEBUGVIZ_DIFFCUBEMAP == 1
   return float4(irradiance, 1);
#endif
   //energy conservation
   float3 F = FresnelSchlickRoughness(surface.NdotV, surface.f0, surface.roughness);
   float3 kD = 1.0f - F;
   kD *= 1.0f - surface.metalness;

   float2 envBRDF = TORQUE_TEX2DLOD(BRDFTexture, float4(surface.NdotV, surface.roughness,0,0)).rg;
   specular *= F * envBRDF.x + surface.f90 * envBRDF.y;
   irradiance *= kD * surface.baseColor.rgb;

   //AO
   irradiance *= surface.ao;
   specular *= computeSpecOcclusion(surface.NdotV, surface.ao, surface.roughness);

   //http://marmosetco.tumblr.com/post/81245981087
   float horizonOcclusion = 1.3;
   float horizon = saturate( 1 + horizonOcclusion * dot(surface.R, surface.N));
   horizon *= horizon;
   
   if(isCapturing == 1)
      return float4(lerp((irradiance + specular* horizon), surface.baseColor.rgb,surface.metalness),0);
   else
      return float4((irradiance + specular* horizon)*ambientColor, 0);//alpha writes disabled
}
