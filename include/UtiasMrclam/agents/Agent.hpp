#pragma once

#include <ostream>
namespace Data {

class Agent {
public:
  enum class Type { ROBOT, LANDMARK };

  Agent(unsigned short id, unsigned short barcode, Type type);
  Agent() = default;
  Agent(Agent &&) = default;
  Agent(const Agent &) = default;
  Agent &operator=(Agent &&) = default;
  Agent &operator=(const Agent &) = default;
  virtual ~Agent() = default;

  struct Identifier {
  protected:
    friend Agent;

    unsigned short value_;

  public:
    Identifier() = default;
    Identifier(unsigned short identifier) : value_{identifier} {};

    bool operator==(const Identifier &other) const {
      return value_ == other.value_;
    }

    bool operator<(const Identifier &other) const {
      return value_ < other.value_;
    }

    operator std::string() const { return std::to_string(value_); }

    friend std::ostream &operator<<(std::ostream &os,
                                    const Identifier &identifier) {
      return os << identifier.value_;
    }

    friend std::string operator+(const std::string &s,
                                 const Identifier &identifier) {
      return s + std::to_string(identifier.value_);
    }
  };

  struct ID : Identifier {};
  struct Barcode : Identifier {};

  const ID id() const;
  const Barcode barcode() const;
  const Type type() const;

private:
  /**
   * @brief Numerical identifier for the robot.
   * @note The handler starts this index at 1. Therefore the first robot has an
   * ID of 1.
   */
  ID id_;

  /**
   * @brief  Barcode associated with the robot. This is what the other robots
   * will read during there operation to identify each other.
   */
  Barcode barcode_;

  Type type_;
};

} // namespace Data
