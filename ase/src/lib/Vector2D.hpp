class Vector2D
{
private:
  /* data */
public:
  Vector2D(float _x = 0.0, float _y = 0.0)
    : x(_x)
    , y(_y){};
  ~Vector2D(){};

  float x;
  float y;

  Vector2D& operator+=(const Vector2D& rhs)
  {
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
  };
  Vector2D& operator-=(const Vector2D& rhs)
  {
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
  };

  friend Vector2D operator-(Vector2D lhs, const Vector2D& rhs)
  {
    return Vector2D(lhs.x - rhs.x, lhs.y - rhs.y);
  };
  friend Vector2D operator+(Vector2D lhs, const Vector2D& rhs)
  {
    return Vector2D(lhs.x + rhs.x, lhs.y + rhs.y);
  };

  friend float operator*(Vector2D lhs, const Vector2D& rhs)
  {
    return lhs.x * rhs.x + lhs.y * rhs.y;
  };
  friend Vector2D operator*(Vector2D p, const float s)
  {
    p.x *= s;
    p.y *= s;
    return p;
  };
  friend Vector2D operator/(Vector2D p, const float s)
  {
    p.x /= s;
    p.y /= s;
    return p;
  };

  float sqLength() { return this->x * this->x + this->y * this->y; };
};