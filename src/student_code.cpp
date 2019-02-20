#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"

#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {
    // Perform linear interpolation on type T variables x and y and parameter t,
    // assuming that type T implements addition overload.
    template<class T>
    inline T lerp(const T &x, const T &y, double t) {
        return x * t + y * (1 - t);
    }

    void BezierCurve::evaluateStep() {
        // Part 1.
        // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
        // Store all of the intermediate control points into the 2D vector evaluatedLevels.
        if (evaluatedLevels.back().size() == 1)
            return;
        std::vector<Vector2D> &oldLevel = evaluatedLevels.back();
        std::vector<Vector2D> newLevel;
        for (int i = 0; i < evaluatedLevels.back().size() - 1; i++)
            newLevel.push_back(lerp(oldLevel[i], oldLevel[i + 1], t));
        evaluatedLevels.push_back(newLevel);
    }


    Vector3D BezierPatch::evaluate(double u, double v) const {
        // Part 2.
        // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
        // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
        // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)
        std::vector<Vector3D> rowResult;
        for (const std::vector<Vector3D> &controlPoint : controlPoints)
            rowResult.push_back(evaluate1D(controlPoint, u));
        return evaluate1D(rowResult, v);
    }

    Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const {
        // Part 2.
        // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
        // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.
        std::vector<Vector3D> lastLevel(points);
        for (int _ = 0; _ < points.size() - 1; _++) {
            std::vector<Vector3D> newLevel;
            for (int i = 0; i < lastLevel.size() - 1; i++)
                newLevel.push_back(lerp(lastLevel[i], lastLevel[i + 1], t));
            lastLevel = newLevel;
        }
        return lastLevel[0];
    }


    Vector3D Vertex::normal(void) const {
        // Part 3.
        // Returns an approximate unit normal at this vertex, computed by
        // taking the area-weighted average of the normals of neighboring
        // triangles, then normalizing.

        Vector3D result(0, 0, 0);
        HalfedgeCIter h = halfedge();
        do {
            result += h->face()->normal();
            h = h->twin()->next();
        } while (h != halfedge());
        return result.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // Part 4.
        // This method should flip the given edge and return an iterator to the flipped edge.
        if (e0->isBoundary())
            return e0;

        // Getting old halfedges
        HalfedgeIter h0_old = e0->halfedge(), h1_old = h0_old->twin();
        HalfedgeIter h0_next = h0_old->next(), h1_next = h1_old->next();
        HalfedgeIter h0_prev = h0_next->next(), h1_prev = h1_next->next();

        // Creating new halfedges
        HalfedgeIter h0_new = newHalfedge(), h1_new = newHalfedge();
        e0->halfedge() = h0_new;

        // Setting new face 0
        h0_new->setNeighbors(h1_prev, h1_new, h0_prev->vertex(), e0, h1_prev->face());
        h1_prev->next() = h0_next;
        h0_next->next() = h0_new;
        h0_next->face() = h1_prev->face();
        h1_prev->face()->halfedge() = h0_new;
        h0_next->vertex()->halfedge() = h0_next;

        // Setting new face 1
        h1_new->setNeighbors(h0_prev, h0_new, h1_prev->vertex(), e0, h0_prev->face());
        h0_prev->next() = h1_next;
        h1_next->next() = h1_new;
        h1_next->face() = h0_prev->face();
        h0_prev->face()->halfedge() = h1_new;
        h1_next->vertex()->halfedge() = h1_next;

        // Removing old halfedges
        deleteHalfedge(h0_old);
        deleteHalfedge(h1_old);

        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 5.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        return newVertex();
    }


    void MeshResampler::upsample(HalfedgeMesh &mesh) {
        // TODO Part 6.
        // This routine should increase the number of triangles in the mesh using Loop subdivision.
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.


        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.


        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.


        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)


        // TODO Now flip any new edge that connects an old and new vertex.


        // TODO Finally, copy the new vertex positions into final Vertex::position.

        return;
    }
}

#pragma clang diagnostic pop