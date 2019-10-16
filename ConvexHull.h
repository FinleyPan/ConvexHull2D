#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include <opencv2/core/core.hpp>

template <typename T>
class ConvexHull2D {

public:    
    typedef cv::Point_<T> VecType;
    enum VertexPosition {
        kInside = -1,
        kAtEdge,
        kOutside,
    };
    ConvexHull2D();
    ConvexHull2D(const std::vector<VecType>& vertices);
    ConvexHull2D(const ConvexHull2D& right);
    ConvexHull2D& operator=(const ConvexHull2D& right);
    ~ConvexHull2D();

    bool ExpandAndUpdate(const VecType& vertex);
    bool ExpandAndUpdate(const std::vector<VecType>& vertices);
    VertexPosition LocateVertex(const VecType& vertex,float& distance) const;
    inline const std::vector<VecType>& vertices() const {return vertices_;}

private:
    static bool Compare(const VecType& vert1, const VecType& vert2);
    static void SolveLine(const VecType& vert1, const VecType& vert2,double& a,double& b, double& c);
    static VecType base;
    bool GrahamScan();
    double Cosine(const VecType& vert) const;

    std::vector<VecType> vertices_;
    std::vector<double> polar_angles_;
    VecType base_axis_, centroid_;
};

//--------------------definitions for static members---------------------------
template <typename T>
typename ConvexHull2D<T>::VecType ConvexHull2D<T>::base;

template <typename T>
bool ConvexHull2D<T>::Compare(const VecType& vert1, const VecType& vert2) {
    VecType vec1 = vert1 - base;
    VecType vec2 = vert2 - base;
    float cross_product = vec1.cross(vec2);
    if(std::fabs(cross_product) < 1e-6) {
        return std::sqrt(vec1.dot(vec1) < vec2.dot(vec2)) ;
    }
    else {
        return cross_product > 0;
    }
}

template <typename T>
void ConvexHull2D<T>::SolveLine(const VecType& vert1, const VecType& vert2,double& a,double& b, double& c){
    double dx = vert1.x - vert2.x;
    double dy = vert1.y - vert2.y;
    a = dy;
    b = -dx;
    c = dy * vert1.x - dx * vert1.y;
}

//--------------------definitions for non-static members--------------------------
template <typename T>
ConvexHull2D<T>::ConvexHull2D() {

}

template <typename T>
ConvexHull2D<T>::ConvexHull2D(const std::vector<VecType>& vertices) : vertices_(vertices) {
    auto end_unique = std::unique(vertices_.begin(),vertices_.end());
    vertices_.erase(end_unique,vertices_.end());

    if(vertices_.size() >= 3)
        GrahamScan();
    else
        vertices_.clear();
}

template <typename T>
ConvexHull2D<T>::ConvexHull2D(const ConvexHull2D<T>& right) : vertices_(right.vertices_),
                       polar_angles_(right.polar_angles_), base_axis_(right.base_axis_) {

}

template <typename T>
ConvexHull2D<T>& ConvexHull2D<T>::operator=(const ConvexHull2D<T>& right) {
    if(this != &right) {
        vertices_ = right.vertices_;
        polar_angles_ = right.polar_angles_;
        base_axis_ = right.base_axis_;
    }
    return *this;
}

template <typename T>
ConvexHull2D<T>::~ConvexHull2D<T>() {

}

template <typename T>
bool ConvexHull2D<T>::ExpandAndUpdate(const VecType& vertex) {
    if(vertices_.empty())
        return false;
    float distance;
    bool expanded;

    switch(LocateVertex(vertex,distance)) {
    case kInside :
    case kAtEdge :
        expanded = false;
        break;
    case kOutside:
        vertices_.push_back(vertex);
        GrahamScan();
        expanded = true;
        break;
    }

    return expanded;
}

template <typename T>
bool ConvexHull2D<T>::ExpandAndUpdate(const std::vector<VecType>& vertices) {
    if(!vertices_.empty() || (vertices_.empty() && vertices.size() >= 3) ){
        for(const VecType& vert : vertices) {
            vertices_.push_back(vert);
        }
        auto end_unique = std::unique(vertices_.begin(),vertices_.end());
        vertices_.erase(end_unique,vertices_.end());
        if(vertices_.size() >= 3){
            size_t ori_size = vertices_.size();
            return GrahamScan();
        }else
            return false;
    }else
        return false;
}

template <typename T>
double ConvexHull2D<T>::Cosine(const VecType& vert) const{
    VecType vec = vert - base;
    double len1 = std::sqrt(vec.ddot(vec));
    double len2 = std::sqrt(base_axis_.ddot(base_axis_));
    return vec.ddot(base_axis_) / (len1 * len2);
}

template <typename T>
bool ConvexHull2D<T>::GrahamScan() {
    std::vector<VecType> stack;
    //find base's index
    int index = 0;
    int top = 2;
    for(int i=1; i<vertices_.size(); i++) {
        if(vertices_[i].y < vertices_[index].y ||(vertices_[i].y == vertices_[index].y
           && vertices_[i].x < vertices_[index].x))
            index = i;
    }

    std::swap(vertices_[0],vertices_[index]);
    base = vertices_[0];
    std::sort(vertices_.begin()+1,vertices_.end(),Compare);
    for(int i=0; i<3; i++)
        stack.push_back(vertices_[i]);

    for(int i=3 ; i<vertices_.size() ;i++) {
        while (top > 0 && (vertices_[i]-stack[top-1]).cross(stack[top]-stack[top-1]) >=0)
        {
            --top;
            stack.pop_back();
        }
        stack.push_back(vertices_[i]);
        ++top;
    }

    if(stack.size() < 3) {
        vertices_.clear();
        polar_angles_.clear();
        centroid_ = base_axis_ = VecType(0,0);
        base = VecType(0,0);
        return false;
    }

    std::swap(vertices_,stack);
    centroid_ = VecType(0,0);
    base_axis_ = vertices_[1] - vertices_[0];
    polar_angles_.resize(vertices_.size()-1, 1.0f);
    int i = -1;
    for(const VecType& vert : vertices_) {
        centroid_ += vert;
        if(++i > 1) {
            polar_angles_[i-1] = Cosine(vert);
        }
    }
    centroid_ /= static_cast<double>(vertices_.size());
    return true;
}

template <typename T>
typename ConvexHull2D<T>::VertexPosition
ConvexHull2D<T>::LocateVertex(const VecType& vertex,float& distance) const{
    VecType vec = vertex - vertices_.front();
    VecType end_axis = vertices_.back() - vertices_.front();
    double base2vec = base_axis_.cross(vec);
    double vec2end = vec.cross(end_axis);
    if(std::fabs(base2vec) < 1e-6) {
        VecType vec1 = vertex - vertices_[1];
        double dot_product = vec1.dot(vec);
        if(dot_product < 0 || std::fabs(dot_product) < 1e-6) {
            distance = 0.0f;
            return kAtEdge;
        }else {
            distance = std::sqrt((vertex - centroid_).dot(vertex - centroid_));
            return kOutside;
        }
    }else if(std::fabs(vec2end) < 1e-6) {
        VecType vec1 = vertex - vertices_.rbegin()[0];
        VecType vec2 = vertex - vertices_.rbegin()[1];
        double dot_product = vec1.dot(vec2);
        if(dot_product < 0 || std::fabs(dot_product) < 1e-6) {
            distance = 0.0f;
            return kAtEdge;
        }else {
            distance = std::sqrt((vertex - centroid_).dot(vertex - centroid_));
            return kOutside;
        }
    }else if(base2vec < 0 || vec2end < 0) {
        distance = std::sqrt((vertex - centroid_).dot(vertex - centroid_));
        return kOutside;
    }else {
        double angle = Cosine(vertex);
        int i = 0;
        for(; i < polar_angles_.size()-1; i++) {
            if(angle < polar_angles_[i] && angle >= polar_angles_[i+1]) {
                break;
            }
        }
        double a1,a2,b1,b2,c1,c2,det;
        VecType intersec;
        SolveLine(vertex,vertices_.front(),a1,b1,c1);
        SolveLine(vertices_[i+1],vertices_[i+2],a2,b2,c2);
        det = a1 * b2 - a2 * b1;
        intersec.x = (c1*b2 - c2*b1) / det;
        intersec.y = (a1*c2 - a2*c1) / det;

        VecType vec1 = intersec - vertex;
        VecType vec2 = intersec - vertices_.front();
        double dot_product = vec1.dot(vec2);
        if(dot_product > 1e-6){
            distance = 0.0f;
            return kInside;
        }else{
            if(std::fabs(dot_product) <= 1e-6){
                distance = 0.0f;
                return kAtEdge;
            }else{
                distance = std::sqrt((vertex - centroid_).dot(vertex - centroid_));
                return kOutside;
            }

        }
    }
}

#endif
