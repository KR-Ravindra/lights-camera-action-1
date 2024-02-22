#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>
using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }

    
}
    
Curve evalBezier(const vector< Vector3f >& P, unsigned steps) {
    // Check
    if (P.size() < 4 || P.size() % 3 != 1) {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit(0);
    }

    // Create the curve
    Curve curve;

    // Initialize the previous tangent
    Vector3f prevT;

    // For each segment
    for (unsigned i = 0; i <= P.size() - 4; i += 3) {
        // For each step
        for (unsigned step = 0; step <= steps; ++step) {
            float t = float(step) / steps;

            // Calculate the position
            Vector3f V = pow(1 - t, 3) * P[i]
                        + 3 * pow(1 - t, 2) * t * P[i + 1]
                        + 3 * (1 - t) * pow(t, 2) * P[i + 2]
                        + pow(t, 3) * P[i + 3];

            // Calculate the tangent
            Vector3f T = -3 * pow(1 - t, 2) * P[i]
                        + (3 * pow(1 - t, 2) - 6 * t * (1 - t)) * P[i + 1]
                        + (6 * t * (1 - t) - 3 * pow(t, 2)) * P[i + 2]
                        + 3 * pow(t, 2) * P[i + 3];
            T.normalize();

            // Calculate the normal and binormal
            Vector3f N;
            Vector3f B;
            if (step == 0) {
                // Use the derivative of the tangent to calculate the normal
                Vector3f dT = 6 * (1 - t) * P[i]
                            - 6 * (1 - 2 * t) * P[i + 1]
                            + 6 * (2 * t - 1) * P[i + 2]
                            + 6 * t * P[i + 3];
                N = Vector3f::cross(T, dT);
                B = Vector3f::cross(T, N);
                prevT = T;
            } else {
                N = Vector3f::cross(prevT, T);
                B = Vector3f::cross(T, N);
                prevT = T;
            }
            N.normalize();
            B.normalize();

            // Add the point to the curve
            curve.push_back(CurvePoint{V, T, N, B});
        }
    }

    return curve;
}

Curve evalBspline(const vector< Vector3f >& P, unsigned steps) {
    // Check
    if (P.size() < 4) {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit(0);
    }

    // Create the curve
    Curve curve;

    // Define the B-spline to Bezier matrix
    Matrix4f bsplineToBezier;
    bsplineToBezier(0, 0) = -1/6.0; bsplineToBezier(0, 1) = 3/6.0; bsplineToBezier(0, 2) = -3/6.0; bsplineToBezier(0, 3) = 1/6.0;
    bsplineToBezier(1, 0) = 3/6.0; bsplineToBezier(1, 1) = -6/6.0; bsplineToBezier(1, 2) = 3/6.0; bsplineToBezier(1, 3) = 0;
    bsplineToBezier(2, 0) = -3/6.0; bsplineToBezier(2, 1) = 3/6.0; bsplineToBezier(2, 2) = 0; bsplineToBezier(2, 3) = 0;
    bsplineToBezier(3, 0) = 1/6.0; bsplineToBezier(3, 1) = 0; bsplineToBezier(3, 2) = 0; bsplineToBezier(3, 3) = 0;

    // For each segment
    for (unsigned i = 0; i <= P.size() - 4; ++i) {
        // Convert the control points to Bezier basis
        vector<Vector3f> bezierControlPoints;
        for (unsigned j = 0; j < 4; ++j) {
            Vector4f controlPoint(P[i + j], 1);
            Vector4f bezierControlPoint = bsplineToBezier * controlPoint;
            bezierControlPoints.push_back(Vector3f(bezierControlPoint[0], bezierControlPoint[1], bezierControlPoint[2]));
        }

        // Evaluate the Bezier curve
        Curve bezierCurve = evalBezier(bezierControlPoints, steps);

        // Add the points to the curve
        curve.insert(curve.end(), bezierCurve.begin(), bezierCurve.end());
    }

    return curve;
}
Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).
    
    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );
        
        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );
        
        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}

void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING ); 
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );
    
    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            glColor3f( 0, 0, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }
    
    // Pop state
    glPopAttrib();
}
