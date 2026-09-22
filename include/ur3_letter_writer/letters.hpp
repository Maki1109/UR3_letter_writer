#pragma once
#include <string>
#include <utility>
#include <vector>

namespace ur3_letter_writer
{

// Toa do chuan hoa (u, v) trong hinh vuong don vi [0,1] x [0,1]
// u: truc ngang (trai -> phai), v: truc doc (duoi -> tren)
using Point2D = std::pair<double, double>;

// Mot net but lien mach (khong nhac dau cong tac len giua cac diem)
using Stroke = std::vector<Point2D>;

// Mot chu cai = danh sach cac net. Giua 2 net, dau cong tac se duoc
// nhac len (pen up) truoc khi di chuyen sang net moi.
using Letter = std::vector<Stroke>;

// Tra ve danh sach net cho 1 ky tu (khong phan biet hoa/thuong).
// Nem std::invalid_argument neu ky tu chua duoc dinh nghia.
Letter getLetterStrokes(char c);

// Danh sach cac ky tu hien dang duoc ho tro (dung de in thong bao/help).
std::string supportedLetters();

}  // namespace ur3_letter_writer
