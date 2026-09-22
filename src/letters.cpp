#include "ur3_letter_writer/letters.hpp"

#include <cctype>
#include <cmath>
#include <stdexcept>

namespace ur3_letter_writer
{

namespace
{

// Sinh cac diem tren mot cung ELIP (tam (cx,cy), ban truc rx/ry) tu goc
// start_deg toi end_deg (ke ca 2 dau), chia deu thanh num_points diem.
// Dung de tao net cong muot cho cac chu co duong cong (vd chu S), thay vi
// go tay tung toa do.
Stroke arcPoints(
  double cx, double cy, double rx, double ry,
  double start_deg, double end_deg, int num_points)
{
  Stroke pts;
  pts.reserve(static_cast<size_t>(num_points));
  const double to_rad = M_PI / 180.0;
  for (int i = 0; i < num_points; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
    const double a = (start_deg + t * (end_deg - start_deg)) * to_rad;
    pts.emplace_back(cx + rx * std::cos(a), cy + ry * std::sin(a));
  }
  return pts;
}

// Chu 'S' = 2 cung elip noi tiep nhau tai tam o (0.5, 0.5):
//   - Bung TREN : elip tam (0.5, 0.75), quet NGUOC chieu kim dong ho tu
//                 goc 20 do (dau phai phia tren) qua dinh, sang trai, xuong
//                 den 270 do (diem duoi cung cua elip = tam chu).
//   - Bung DUOI : elip tam (0.5, 0.25), quet THEO chieu kim dong ho tu
//                 90 do (diem tren cung = tam chu) sang phai, xuong day,
//                 ket thuc o -160 do (duoi ben trai).
// Hai cung tiep xuc nhau tai (0.5, 0.5) nen net lien mach, khong gay khuc.
Stroke makeS(int points_per_arc)
{
  Stroke upper = arcPoints(0.5, 0.75, 0.5, 0.25, 20.0, 270.0, points_per_arc);
  Stroke lower = arcPoints(0.5, 0.25, 0.5, 0.25, 90.0, -160.0, points_per_arc);
  // Bo diem dau cua cung duoi vi trung voi diem cuoi cung tren (0.5, 0.5)
  upper.insert(upper.end(), lower.begin() + 1, lower.end());
  return upper;
}

}  // namespace


// Moi chu duoc thiet ke trong o vuong don vi [0,1] x [0,1]:
//   u = 0 -> trai,  u = 1 -> phai
//   v = 0 -> duoi,  v = 1 -> tren
// Sau nay node se scale + dich chuyen sang mat phang Cartesian thuc te.
Letter getLetterStrokes(char c)
{
  c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));

  switch (c) {
    case 'A':
      return {
        {{0.00, 0.00}, {0.50, 1.00}},                       // net trai (di len)
        {{0.50, 1.00}, {1.00, 0.00}},                       // net phai (di xuong)
        {{0.22, 0.38}, {0.78, 0.38}},                       // thanh ngang
      };

    case 'B':
      return {
        {{0.00, 0.00}, {0.00, 1.00}},                       // song lung doc
        {{0.00, 1.00}, {0.65, 0.92}, {0.65, 0.58}, {0.00, 0.50}},  // bung tren (xap xi cung)
        {{0.00, 0.50}, {0.75, 0.42}, {0.75, 0.08}, {0.00, 0.00}},  // bung duoi (xap xi cung)
      };

    case 'M':
      return {
        {{0.00, 0.00}, {0.00, 1.00}, {0.50, 0.40}, {1.00, 1.00}, {1.00, 0.00}},
      };

    case 'N':
      return {
        {{0.00, 0.00}, {0.00, 1.00}, {1.00, 0.00}, {1.00, 1.00}},
      };

    case 'S':
      // Mot net cong lien mach (khong nhac dau cong tac giua chung).
      // 18 diem/cung -> 35 diem, du muot de computeCartesianPath noi tron.
      return {
        makeS(18),
      };

    case 'T':
      return {
        {{0.00, 1.00}, {1.00, 1.00}},                       // thanh ngang tren
        {{0.50, 1.00}, {0.50, 0.00}},                       // than doc
      };

    case 'L':
      return {
        {{0.00, 1.00}, {0.00, 0.00}, {1.00, 0.00}},
      };

    case 'I':
      return {
        {{0.50, 0.00}, {0.50, 1.00}},
      };

    default:
      throw std::invalid_argument(
        std::string("Chu '") + c + "' chua duoc dinh nghia. "
        "Hay them mot 'case' moi trong src/letters.cpp voi danh sach net (u,v).");
  }
}

std::string supportedLetters()
{
  return "A, B, I, L, M, N, S, T  (them chu moi bang cach sua src/letters.cpp)";
}

}  // namespace ur3_letter_writer
