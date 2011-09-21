float4x4 mtxWorld;
float4x4 mtxView;
float4x4 mtxProjection;
float4x4 mtxWorldViewProjection;
float4x4 mtxLightWorldViewProjection;
float4x4 mtxLightView;
float4x4 mtxLightProjection;

float3 mainLightPosition;
float3 mainLightDirection;
float mainLightPower;
float4 ambientLight;
float4 diffuseLight;

texture txtChessBoard;
texture txtShadowMap;

sampler TextureSampler = 
sampler_state 
{ 
texture = <txtChessBoard> ; 
magfilter = LINEAR; 
minfilter = LINEAR; 
mipfilter=LINEAR; 
};

sampler ShadowMapSampler = 
sampler_state 
{ 
texture = <txtShadowMap> ; 
magfilter = POINT; 
minfilter = POINT; 
mipfilter = POINT; 
AddressU = clamp; 
AddressV = clamp;
};

// TODO: add effect parameters here.


float4 GetPositionFromLight(float4 position)
{
	float4x4 WorldViewProjection =
	 mul(mul(mtxWorld, mtxLightView), mtxLightProjection);
	return mul(position, WorldViewProjection);  
}



float DotProduct(float3 lightPos, float3 pos3D, float3 normal)
{
    float3 lightDir = normalize(pos3D - lightPos);
    return dot(-lightDir, normal);    
}

struct VertexShaderInput
{
    float4 Position : POSITION0;
    float3 Normal : NORMAL0;
    float2 TexCoords : TEXCOORD0;
};

struct VertexShaderOutput
{
    float4 Position   : POSITION0;   // vertex position 
    float2 TextureUV  : TEXCOORD0;  // vertex texture coords   
    float3 vNormal	  : TEXCOORD1;
    float4 vPos       : TEXCOORD2;  
};

VertexShaderOutput VertexShaderFunction(VertexShaderInput input)
{
	VertexShaderOutput output;
	//generate the world-view-projection matrix
	float4x4 wvp = mul(mul(mtxWorld, mtxView), mtxProjection);

	//transform the input position to the output
	input.Position.w = 1.0;
	output.Position = mul(input.Position, wvp);

	//transform the normal to world space
	output.vNormal =  mul(input.Normal, mtxWorld);

	//do not transform the position needed for the
	//shadow map determination
	output.vPos = input.Position;

	//pass the texture coordinate as-is
	output.TextureUV = input.TexCoords;

	//return the output structure
	return output;
}

float4 PixelShaderFunction(VertexShaderOutput input) : COLOR0
{
    float4 output;
    
    // Standard lighting equation
    float4 vTotalLightDiffuse = float4(0,0,0,1);
    float3 lightDir = normalize(mainLightPosition-input.vPos);  // direction of light
    vTotalLightDiffuse += diffuseLight * max(0,dot(input.vNormal, lightDir)); 
    vTotalLightDiffuse.a = 1.0f;
	// Now, consult the ShadowMap to see if we're in shadow
    float4 lightingPosition 
		= GetPositionFromLight(input.vPos);// Get our position on the shadow map
   
    // Get the shadow map depth value for this pixel   
    float2 ShadowTexC = 
		0.5 * lightingPosition.xy / lightingPosition.w + float2( 0.5, 0.5 );
    ShadowTexC.y = 1.0f - ShadowTexC.y;

    float shadowdepth = tex2D(ShadowMapSampler, ShadowTexC).r;    
    
    // Check our value against the depth value
    float ourdepth = 1 - (lightingPosition.z / lightingPosition.w);
    
    // Check the shadowdepth against the depth of this pixel
    // a fudge factor is added to account for floating-point error
	/*
	if (shadowdepth-0.03 > ourdepth)
	{
	    // we're in shadow, cut the light
		vTotalLightDiffuse = float4(0,0,0,1);
	};
	*/
    output = tex2D(TextureSampler, input.TextureUV) * 
		(vTotalLightDiffuse + ambientLight);
        
    return output;
}

technique Standard
{
    pass Pass1
    {
        // TODO: set renderstates here.

        VertexShader = compile vs_2_0 VertexShaderFunction();
        PixelShader = compile ps_2_0 PixelShaderFunction();
    }
}

struct ShadowMapVertexShaderInput
{
    float4 Position : POSITION0;
};

struct ShadowMapVertexShaderOutput
{
	float4 Position : POSITION;
	float Depth : TEXCOORD0;
};

ShadowMapVertexShaderOutput ShadowMapVertexShaderFunction(float4 vPos: POSITION)
{
    ShadowMapVertexShaderOutput output;
	output.Position = GetPositionFromLight(vPos); 
	// Depth is Z/W.  This is returned by the pixel shader.
	// Subtracting from 1 gives us more precision in floating point.
	output.Depth.x = 1-(output.Position.z/output.Position.w);	
	return output;
}

float4 ShadowMapPixelShaderFunction(ShadowMapVertexShaderOutput input) : COLOR
{
	return float4(input.Depth.x,0,0,1);
}

technique ShadowMap
{
    pass Pass1
    {
		// These render states are necessary to get a shadow map.
		// You should consider resetting CullMode and AlphaBlendEnable
		// before you render your main scene.	
		CullMode = NONE;
		ZEnable = TRUE;
		ZWriteEnable = TRUE;
		AlphaBlendEnable = FALSE;

        VertexShader = compile vs_2_0 ShadowMapVertexShaderFunction();
        PixelShader = compile ps_2_0 ShadowMapPixelShaderFunction();
    }
}

