#pragma clang diagnostic push
#pragma ide diagnostic ignored "modernize-use-auto"

#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {
    template<class T>
    inline T lerp(const T &x, const T &y, double t) {
        // Perform linear interpolation on type T variables x and y and parameter t,
        // assuming that type T implements addition overload.
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
            Vector3D side0 = h->next()->vertex()->position - h->vertex()->position;
            Vector3D side1 = h->next()->next()->vertex()->position - h->vertex()->position;
            result += cross(side0, side1);
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
        // Part 5.
        // This method should split the given edge and return an iterator to the newly inserted vertex.
        // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
        if (e0->isBoundary())
            return e0->halfedge()->vertex();

        // Getting old halfedges
        HalfedgeIter h0_old = e0->halfedge(), h1_old = h0_old->twin();
        HalfedgeIter h0_next = h0_old->next(), h1_next = h1_old->next();
        HalfedgeIter h0_prev = h0_next->next(), h1_prev = h1_next->next();

        HalfedgeIter bs[] = {h0_next, h0_prev, h1_next, h1_prev};

        FaceIter f0_old = h0_old->face(), f1_old = h1_old->face();

        // Creating new halfedges, edges and faces
        HalfedgeIter hs[] = {newHalfedge(), newHalfedge(), newHalfedge(), newHalfedge()};
        HalfedgeIter ps[] = {newHalfedge(), newHalfedge(), newHalfedge(), newHalfedge()};
        EdgeIter es[] = {e0, newEdge(), newEdge(), newEdge()};
        FaceIter fs[] = {f0_old, f1_old, newFace(), newFace()};

        // Creating new vertex
        VertexIter m = newVertex();
        m->position = (h0_old->vertex()->position + h1_old->vertex()->position) / 2;
        m->halfedge() = hs[0];

        // Editing face by face
        for (int i = 0; i < 4; i++) {
            // Setting new halfedges
            hs[i]->setNeighbors(bs[i], ps[(i + 3) % 4], m, es[i], fs[i]);
            ps[i]->setNeighbors(hs[i], hs[(i + 1) % 4], bs[(i + 1) % 4]->vertex(), es[(i + 1) % 4], fs[i]);
            bs[i]->next() = ps[i];
            bs[i]->face() = fs[i];

            // Setting new edges and faces
            es[i]->halfedge() = hs[i];
            fs[i]->halfedge() = hs[i];

            // Handling old vertex
            ps[i]->vertex()->halfedge() = ps[i];
        }

        // Removing old halfedges, edge and faces
        deleteHalfedge(h0_old);
        deleteHalfedge(h1_old);

        return m;
    }


    Vector3D subdivNewLocation(const VertexIter v) {
        // calculating the new location for a old vertex v using subdivision rules
        int n = 0;
        Vector3D result = Vector3D(0, 0, 0);
        HalfedgeIter start = v->halfedge();
        HalfedgeIter h = v->halfedge();

        do {
            n++;
            result += h->next()->vertex()->position;
            h = h->twin()->next();
        } while (h != start);

        double u = (n == 3) ? 0.1875 : (0.375 / n);

        return result * u + (1 - n * u) * v->position;
    }

    Vector3D subdivNewLocation(const EdgeIter v) {
        // calculating the new location for new edge midpoint using subdivision rules
        HalfedgeCIter h = v->halfedge();
        HalfedgeCIter t = h->twin();
        return 0.375 * (h->vertex()->position + t->vertex()->position)
               + 0.125 * (h->next()->next()->vertex()->position + t->next()->next()->vertex()->position);
    }

    void MeshResampler::upsample(HalfedgeMesh &mesh) {
        // Part 6.
        // This routine should increase the number of triangles in the mesh
        // using Loop subdivision.

        // Compute new positions for all the vertices in the input mesh, using
        // the loop subdivision rule and store them in Vertex::newPosition.
        // Also mark each vertex as being a vertex of the original mesh.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->isNew = false;
            v->newPosition = subdivNewLocation(v);
        }

        // Compute the updated vertex positions associated with edges, and
        // store it in Edge::newPosition. Also mark each edge as being en edge
        // of the original mesh.
        EdgeIter oldEdgeBegin = mesh.edgesBegin(), oldEdgeEnd = mesh.edgesEnd();
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            e->isNew = false;
            e->newPosition = subdivNewLocation(e);
        }

        // Split every original edge in the mesh in any order.
        EdgeIter e = mesh.edgesBegin();
        Size oldNumEdges = mesh.nEdges();
        for (int _ = 0; _ < oldNumEdges; _++) {
            Vector3D newPosition = e->newPosition;
            VertexIter midpoint = mesh.splitEdge(e);
            // Mark new vertex
            midpoint->isNew = true;
            midpoint->newPosition = newPosition;
            // Mark new edges. Note that the two edges that came from a split
            // of an original edge count as an old one. This is used to
            // determine which edge to flip
            HalfedgeIter h = midpoint->halfedge();
            bool flag = false;
            do {
                h->edge()->isNew = flag;
                flag = !flag;
                h = h->twin()->next();
            } while (h != midpoint->halfedge());

            e++;
        }

        // Flip any new edge that connects an old and new vertex.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            VertexIter u = e->halfedge()->vertex();
            VertexIter v = e->halfedge()->twin()->vertex();
            if (e->isNew && (u->isNew ^ v->isNew)) {
                mesh.flipEdge(e->halfedge()->edge());
            }
        }

        // Copy the new vertex positions into final Vertex::position.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
            v->position = v->newPosition;
        }
    }
}

#pragma clang diagnostic pop