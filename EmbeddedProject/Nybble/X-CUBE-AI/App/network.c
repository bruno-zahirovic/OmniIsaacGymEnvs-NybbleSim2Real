/**
  ******************************************************************************
  * @file    network.c
  * @author  AST Embedded Analytics Research Platform
  * @date    Mon Aug 26 20:58:26 2024
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */


#include "network.h"
#include "network_data.h"

#include "ai_platform.h"
#include "ai_platform_interface.h"
#include "ai_math_helpers.h"

#include "core_common.h"
#include "core_convert.h"

#include "layers.h"



#undef AI_NET_OBJ_INSTANCE
#define AI_NET_OBJ_INSTANCE g_network
 
#undef AI_NETWORK_MODEL_SIGNATURE
#define AI_NETWORK_MODEL_SIGNATURE     "0x6c27d7d8a9e2eb3e46a8a13c5eed891a"

#ifndef AI_TOOLS_REVISION_ID
#define AI_TOOLS_REVISION_ID     ""
#endif

#undef AI_TOOLS_DATE_TIME
#define AI_TOOLS_DATE_TIME   "Mon Aug 26 20:58:26 2024"

#undef AI_TOOLS_COMPILE_TIME
#define AI_TOOLS_COMPILE_TIME    __DATE__ " " __TIME__

#undef AI_NETWORK_N_BATCHES
#define AI_NETWORK_N_BATCHES         (1)

static ai_ptr g_network_activations_map[1] = AI_C_ARRAY_INIT;
static ai_ptr g_network_weights_map[1] = AI_C_ARRAY_INIT;



/**  Array declarations section  **********************************************/
/* Array#0 */
AI_ARRAY_OBJ_DECLARE(
  observations_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 36, AI_STATIC)

/* Array#1 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#2 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_1_Elu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#3 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#4 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_3_Elu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#5 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#6 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_5_Elu_output_0_output_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#7 */
AI_ARRAY_OBJ_DECLARE(
  mus_output_array, AI_ARRAY_FORMAT_FLOAT|AI_FMT_FLAG_IS_IO,
  NULL, NULL, 8, AI_STATIC)

/* Array#8 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 9216, AI_STATIC)

/* Array#9 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 256, AI_STATIC)

/* Array#10 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 32768, AI_STATIC)

/* Array#11 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 128, AI_STATIC)

/* Array#12 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8192, AI_STATIC)

/* Array#13 */
AI_ARRAY_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 64, AI_STATIC)

/* Array#14 */
AI_ARRAY_OBJ_DECLARE(
  mus_weights_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 512, AI_STATIC)

/* Array#15 */
AI_ARRAY_OBJ_DECLARE(
  mus_bias_array, AI_ARRAY_FORMAT_FLOAT,
  NULL, NULL, 8, AI_STATIC)

/**  Tensor declarations section  *********************************************/
/* Tensor #0 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_weights, AI_STATIC,
  0, 0x0,
  AI_SHAPE_INIT(4, 128, 64, 1, 1), AI_STRIDE_INIT(4, 4, 512, 32768, 32768),
  1, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_weights_array, NULL)

/* Tensor #1 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_bias, AI_STATIC,
  1, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_bias_array, NULL)

/* Tensor #2 */
AI_TENSOR_OBJ_DECLARE(
  observations_output, AI_STATIC,
  2, 0x0,
  AI_SHAPE_INIT(4, 1, 36, 1, 1), AI_STRIDE_INIT(4, 4, 4, 144, 144),
  1, &observations_output_array, NULL)

/* Tensor #3 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_output, AI_STATIC,
  3, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_output_array, NULL)

/* Tensor #4 */
AI_TENSOR_OBJ_DECLARE(
  mus_weights, AI_STATIC,
  4, 0x0,
  AI_SHAPE_INIT(4, 64, 8, 1, 1), AI_STRIDE_INIT(4, 4, 256, 2048, 2048),
  1, &mus_weights_array, NULL)

/* Tensor #5 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_1_Elu_output_0_output, AI_STATIC,
  5, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_model_actor_mlp_actor_mlp_1_Elu_output_0_output_array, NULL)

/* Tensor #6 */
AI_TENSOR_OBJ_DECLARE(
  mus_bias, AI_STATIC,
  6, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &mus_bias_array, NULL)

/* Tensor #7 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_output, AI_STATIC,
  7, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_output_array, NULL)

/* Tensor #8 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_3_Elu_output_0_output, AI_STATIC,
  8, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_model_actor_mlp_actor_mlp_3_Elu_output_0_output_array, NULL)

/* Tensor #9 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_output, AI_STATIC,
  9, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_output_array, NULL)

/* Tensor #10 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_5_Elu_output_0_output, AI_STATIC,
  10, 0x0,
  AI_SHAPE_INIT(4, 1, 64, 1, 1), AI_STRIDE_INIT(4, 4, 4, 256, 256),
  1, &_model_actor_mlp_actor_mlp_5_Elu_output_0_output_array, NULL)

/* Tensor #11 */
AI_TENSOR_OBJ_DECLARE(
  mus_output, AI_STATIC,
  11, 0x0,
  AI_SHAPE_INIT(4, 1, 8, 1, 1), AI_STRIDE_INIT(4, 4, 4, 32, 32),
  1, &mus_output_array, NULL)

/* Tensor #12 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_weights, AI_STATIC,
  12, 0x0,
  AI_SHAPE_INIT(4, 36, 256, 1, 1), AI_STRIDE_INIT(4, 4, 144, 36864, 36864),
  1, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_weights_array, NULL)

/* Tensor #13 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_bias, AI_STATIC,
  13, 0x0,
  AI_SHAPE_INIT(4, 1, 256, 1, 1), AI_STRIDE_INIT(4, 4, 4, 1024, 1024),
  1, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_bias_array, NULL)

/* Tensor #14 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_weights, AI_STATIC,
  14, 0x0,
  AI_SHAPE_INIT(4, 256, 128, 1, 1), AI_STRIDE_INIT(4, 4, 1024, 131072, 131072),
  1, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_weights_array, NULL)

/* Tensor #15 */
AI_TENSOR_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_bias, AI_STATIC,
  15, 0x0,
  AI_SHAPE_INIT(4, 1, 128, 1, 1), AI_STRIDE_INIT(4, 4, 4, 512, 512),
  1, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_bias_array, NULL)



/**  Layer declarations section  **********************************************/


AI_TENSOR_CHAIN_OBJ_DECLARE(
  mus_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_5_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &mus_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &mus_weights, &mus_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  mus_layer, 7,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &mus_chain,
  NULL, &mus_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _model_actor_mlp_actor_mlp_5_Elu_output_0_nl_params_data[] = { 1.0 };
AI_ARRAY_OBJ_DECLARE(
    _model_actor_mlp_actor_mlp_5_Elu_output_0_nl_params, AI_ARRAY_FORMAT_FLOAT,
    _model_actor_mlp_actor_mlp_5_Elu_output_0_nl_params_data, _model_actor_mlp_actor_mlp_5_Elu_output_0_nl_params_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_5_Elu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_5_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_5_Elu_output_0_layer, 6,
  NL_TYPE, 0x0, NULL,
  nl, forward_elu,
  &_model_actor_mlp_actor_mlp_5_Elu_output_0_chain,
  NULL, &mus_layer, AI_STATIC, 
  .nl_params = &_model_actor_mlp_actor_mlp_5_Elu_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_3_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_weights, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_4_Gemm_output_0_layer, 5,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_model_actor_mlp_actor_mlp_4_Gemm_output_0_chain,
  NULL, &_model_actor_mlp_actor_mlp_5_Elu_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _model_actor_mlp_actor_mlp_3_Elu_output_0_nl_params_data[] = { 1.0 };
AI_ARRAY_OBJ_DECLARE(
    _model_actor_mlp_actor_mlp_3_Elu_output_0_nl_params, AI_ARRAY_FORMAT_FLOAT,
    _model_actor_mlp_actor_mlp_3_Elu_output_0_nl_params_data, _model_actor_mlp_actor_mlp_3_Elu_output_0_nl_params_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_3_Elu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_3_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_3_Elu_output_0_layer, 4,
  NL_TYPE, 0x0, NULL,
  nl, forward_elu,
  &_model_actor_mlp_actor_mlp_3_Elu_output_0_chain,
  NULL, &_model_actor_mlp_actor_mlp_4_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = &_model_actor_mlp_actor_mlp_3_Elu_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_1_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_weights, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_2_Gemm_output_0_layer, 3,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_model_actor_mlp_actor_mlp_2_Gemm_output_0_chain,
  NULL, &_model_actor_mlp_actor_mlp_3_Elu_output_0_layer, AI_STATIC, 
)


AI_STATIC_CONST ai_float _model_actor_mlp_actor_mlp_1_Elu_output_0_nl_params_data[] = { 1.0 };
AI_ARRAY_OBJ_DECLARE(
    _model_actor_mlp_actor_mlp_1_Elu_output_0_nl_params, AI_ARRAY_FORMAT_FLOAT,
    _model_actor_mlp_actor_mlp_1_Elu_output_0_nl_params_data, _model_actor_mlp_actor_mlp_1_Elu_output_0_nl_params_data, 1, AI_STATIC_CONST)
AI_TENSOR_CHAIN_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_1_Elu_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_1_Elu_output_0_output),
  AI_TENSOR_LIST_OBJ_EMPTY,
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_1_Elu_output_0_layer, 2,
  NL_TYPE, 0x0, NULL,
  nl, forward_elu,
  &_model_actor_mlp_actor_mlp_1_Elu_output_0_chain,
  NULL, &_model_actor_mlp_actor_mlp_2_Gemm_output_0_layer, AI_STATIC, 
  .nl_params = &_model_actor_mlp_actor_mlp_1_Elu_output_0_nl_params, 
)

AI_TENSOR_CHAIN_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_chain, AI_STATIC_CONST, 4,
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &observations_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 1, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_output),
  AI_TENSOR_LIST_OBJ_INIT(AI_FLAG_NONE, 2, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_weights, &_model_actor_mlp_actor_mlp_0_Gemm_output_0_bias),
  AI_TENSOR_LIST_OBJ_EMPTY
)

AI_LAYER_OBJ_DECLARE(
  _model_actor_mlp_actor_mlp_0_Gemm_output_0_layer, 1,
  DENSE_TYPE, 0x0, NULL,
  dense, forward_dense,
  &_model_actor_mlp_actor_mlp_0_Gemm_output_0_chain,
  NULL, &_model_actor_mlp_actor_mlp_1_Elu_output_0_layer, AI_STATIC, 
)


#if (AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 204576, 1, 1),
    204576, NULL, NULL),
  AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
    AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1536, 1, 1),
    1536, NULL, NULL),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &observations_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &mus_output),
  &_model_actor_mlp_actor_mlp_0_Gemm_output_0_layer, 0x5e81633d, NULL)

#else

AI_NETWORK_OBJ_DECLARE(
  AI_NET_OBJ_INSTANCE, AI_STATIC,
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 204576, 1, 1),
      204576, NULL, NULL)
  ),
  AI_BUFFER_ARRAY_OBJ_INIT_STATIC(
  	AI_FLAG_NONE, 1,
    AI_BUFFER_INIT(AI_FLAG_NONE,  AI_BUFFER_FORMAT_U8,
      AI_BUFFER_SHAPE_INIT(AI_SHAPE_BCWH, 4, 1, 1536, 1, 1),
      1536, NULL, NULL)
  ),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_IN_NUM, &observations_output),
  AI_TENSOR_LIST_IO_OBJ_INIT(AI_FLAG_NONE, AI_NETWORK_OUT_NUM, &mus_output),
  &_model_actor_mlp_actor_mlp_0_Gemm_output_0_layer, 0x5e81633d, NULL)

#endif	/*(AI_TOOLS_API_VERSION < AI_TOOLS_API_VERSION_1_5)*/



/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_activations(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_activations_map(g_network_activations_map, 1, params)) {
    /* Updating activations (byte) offsets */
    
    observations_output_array.data = AI_PTR(g_network_activations_map[0] + 368);
    observations_output_array.data_start = AI_PTR(g_network_activations_map[0] + 368);
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _model_actor_mlp_actor_mlp_1_Elu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _model_actor_mlp_actor_mlp_1_Elu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _model_actor_mlp_actor_mlp_3_Elu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 512);
    _model_actor_mlp_actor_mlp_3_Elu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 512);
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    _model_actor_mlp_actor_mlp_5_Elu_output_0_output_array.data = AI_PTR(g_network_activations_map[0] + 256);
    _model_actor_mlp_actor_mlp_5_Elu_output_0_output_array.data_start = AI_PTR(g_network_activations_map[0] + 256);
    mus_output_array.data = AI_PTR(g_network_activations_map[0] + 0);
    mus_output_array.data_start = AI_PTR(g_network_activations_map[0] + 0);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_ACTIVATIONS);
  return false;
}




/******************************************************************************/
AI_DECLARE_STATIC
ai_bool network_configure_weights(
  ai_network* net_ctx, const ai_network_params* params)
{
  AI_ASSERT(net_ctx)

  if (ai_platform_get_weights_map(g_network_weights_map, 1, params)) {
    /* Updating weights (byte) offsets */
    
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 0);
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 0);
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 36864);
    _model_actor_mlp_actor_mlp_0_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 36864);
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 37888);
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 37888);
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 168960);
    _model_actor_mlp_actor_mlp_2_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 168960);
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_weights_array.format |= AI_FMT_FLAG_CONST;
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_weights_array.data = AI_PTR(g_network_weights_map[0] + 169472);
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 169472);
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_bias_array.format |= AI_FMT_FLAG_CONST;
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_bias_array.data = AI_PTR(g_network_weights_map[0] + 202240);
    _model_actor_mlp_actor_mlp_4_Gemm_output_0_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 202240);
    mus_weights_array.format |= AI_FMT_FLAG_CONST;
    mus_weights_array.data = AI_PTR(g_network_weights_map[0] + 202496);
    mus_weights_array.data_start = AI_PTR(g_network_weights_map[0] + 202496);
    mus_bias_array.format |= AI_FMT_FLAG_CONST;
    mus_bias_array.data = AI_PTR(g_network_weights_map[0] + 204544);
    mus_bias_array.data_start = AI_PTR(g_network_weights_map[0] + 204544);
    return true;
  }
  AI_ERROR_TRAP(net_ctx, INIT_FAILED, NETWORK_WEIGHTS);
  return false;
}


/**  PUBLIC APIs SECTION  *****************************************************/



AI_DEPRECATED
AI_API_ENTRY
ai_bool ai_network_get_info(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 56072,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .params            = AI_STRUCT_INIT,
      .activations       = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x5e81633d,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}



AI_API_ENTRY
ai_bool ai_network_get_report(
  ai_handle network, ai_network_report* report)
{
  ai_network* net_ctx = AI_NETWORK_ACQUIRE_CTX(network);

  if (report && net_ctx)
  {
    ai_network_report r = {
      .model_name        = AI_NETWORK_MODEL_NAME,
      .model_signature   = AI_NETWORK_MODEL_SIGNATURE,
      .model_datetime    = AI_TOOLS_DATE_TIME,
      
      .compile_datetime  = AI_TOOLS_COMPILE_TIME,
      
      .runtime_revision  = ai_platform_runtime_get_revision(),
      .runtime_version   = ai_platform_runtime_get_version(),

      .tool_revision     = AI_TOOLS_REVISION_ID,
      .tool_version      = {AI_TOOLS_VERSION_MAJOR, AI_TOOLS_VERSION_MINOR,
                            AI_TOOLS_VERSION_MICRO, 0x0},
      .tool_api_version  = AI_STRUCT_INIT,

      .api_version            = ai_platform_api_get_version(),
      .interface_api_version  = ai_platform_interface_api_get_version(),
      
      .n_macc            = 56072,
      .n_inputs          = 0,
      .inputs            = NULL,
      .n_outputs         = 0,
      .outputs           = NULL,
      .map_signature     = AI_MAGIC_SIGNATURE,
      .map_weights       = AI_STRUCT_INIT,
      .map_activations   = AI_STRUCT_INIT,
      .n_nodes           = 0,
      .signature         = 0x5e81633d,
    };

    if (!ai_platform_api_get_network_report(network, &r)) return false;

    *report = r;
    return true;
  }
  return false;
}


AI_API_ENTRY
ai_error ai_network_get_error(ai_handle network)
{
  return ai_platform_network_get_error(network);
}


AI_API_ENTRY
ai_error ai_network_create(
  ai_handle* network, const ai_buffer* network_config)
{
  return ai_platform_network_create(
    network, network_config, 
    AI_CONTEXT_OBJ(&AI_NET_OBJ_INSTANCE),
    AI_TOOLS_API_VERSION_MAJOR, AI_TOOLS_API_VERSION_MINOR, AI_TOOLS_API_VERSION_MICRO);
}


AI_API_ENTRY
ai_error ai_network_create_and_init(
  ai_handle* network, const ai_handle activations[], const ai_handle weights[])
{
  ai_error err;
  ai_network_params params;

  err = ai_network_create(network, AI_NETWORK_DATA_CONFIG);
  if (err.type != AI_ERROR_NONE) {
    return err;
  }
  
  if (ai_network_data_params_get(&params) != true) {
    err = ai_network_get_error(*network);
    return err;
  }
#if defined(AI_NETWORK_DATA_ACTIVATIONS_COUNT)
  /* set the addresses of the activations buffers */
  for (ai_u16 idx=0; activations && idx<params.map_activations.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_activations, idx, activations[idx]);
  }
#endif
#if defined(AI_NETWORK_DATA_WEIGHTS_COUNT)
  /* set the addresses of the weight buffers */
  for (ai_u16 idx=0; weights && idx<params.map_weights.size; idx++) {
    AI_BUFFER_ARRAY_ITEM_SET_ADDRESS(&params.map_weights, idx, weights[idx]);
  }
#endif
  if (ai_network_init(*network, &params) != true) {
    err = ai_network_get_error(*network);
  }
  return err;
}


AI_API_ENTRY
ai_buffer* ai_network_inputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_inputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_buffer* ai_network_outputs_get(ai_handle network, ai_u16 *n_buffer)
{
  if (network == AI_HANDLE_NULL) {
    network = (ai_handle)&AI_NET_OBJ_INSTANCE;
    AI_NETWORK_OBJ(network)->magic = AI_MAGIC_CONTEXT_TOKEN;
  }
  return ai_platform_outputs_get(network, n_buffer);
}


AI_API_ENTRY
ai_handle ai_network_destroy(ai_handle network)
{
  return ai_platform_network_destroy(network);
}


AI_API_ENTRY
ai_bool ai_network_init(
  ai_handle network, const ai_network_params* params)
{
  ai_network* net_ctx = AI_NETWORK_OBJ(ai_platform_network_init(network, params));
  ai_bool ok = true;

  if (!net_ctx) return false;
  ok &= network_configure_weights(net_ctx, params);
  ok &= network_configure_activations(net_ctx, params);

  ok &= ai_platform_network_post_init(network);

  return ok;
}


AI_API_ENTRY
ai_i32 ai_network_run(
  ai_handle network, const ai_buffer* input, ai_buffer* output)
{
  return ai_platform_network_process(network, input, output);
}


AI_API_ENTRY
ai_i32 ai_network_forward(ai_handle network, const ai_buffer* input)
{
  return ai_platform_network_process(network, input, NULL);
}



#undef AI_NETWORK_MODEL_SIGNATURE
#undef AI_NET_OBJ_INSTANCE
#undef AI_TOOLS_DATE_TIME
#undef AI_TOOLS_COMPILE_TIME

