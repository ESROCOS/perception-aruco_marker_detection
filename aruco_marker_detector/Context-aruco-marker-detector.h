#ifndef GENERATED_ASN1SCC_Context_aruco_marker_detector_H
#define GENERATED_ASN1SCC_Context_aruco_marker_detector_H
/*
Code automatically generated by asn1scc tool
*/
#include "dataview-uniq.h"
#include "asn1crt.h"

#ifdef  __cplusplus
extern "C" {
#endif



typedef struct {
    asn1SccT_Int32 capture_device_id;
    asn1SccT_Double marker_size;
    asn1SccT_Int32 draw_augmented_image;
} asn1SccContext_aruco_marker_detector;

flag asn1SccContext_aruco_marker_detector_Equal(const asn1SccContext_aruco_marker_detector* pVal1, const asn1SccContext_aruco_marker_detector* pVal2);

void asn1SccContext_aruco_marker_detector_Initialize(asn1SccContext_aruco_marker_detector* pVal);

#define ERR_CONTEXT_ARUCO_MARKER_DETECTOR_CAPTURE_DEVICE_ID		1  /**/
#define ERR_CONTEXT_ARUCO_MARKER_DETECTOR_MARKER_SIZE		12  /**/
#define ERR_CONTEXT_ARUCO_MARKER_DETECTOR_DRAW_AUGMENTED_IMAGE		23  /**/
flag asn1SccContext_aruco_marker_detector_IsConstraintValid(const asn1SccContext_aruco_marker_detector* pVal, int* pErrCode);

extern const asn1SccContext_aruco_marker_detector aruco_marker_detector_ctxt; 

/* ================= Encoding/Decoding function prototypes =================
 * These functions are placed at the end of the file to make sure all types
 * have been declared first, in case of parameterized ACN encodings
 * ========================================================================= */

 


#ifdef  __cplusplus
}

#endif

#endif