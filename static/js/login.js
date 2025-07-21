// 로그인 팝업 열기
document.getElementById("loginBtn").addEventListener("click", function (e) {
  e.preventDefault();
  document.getElementById("loginPopup").style.display = "block";
});

// 로그인 폼 제출 처리
document.getElementById("loginForm").addEventListener("submit", function (e) {
  e.preventDefault();
  const form = e.target;
  const formData = new FormData(form);

  fetch("/login_check", {
    method: "POST",
    body: formData
  })
  .then(res => res.text())
  .then(result => {
    if (result === "admin" || result === "user") {
      document.querySelector(".login").classList.add("loading");
      setTimeout(() => {
        document.querySelector(".login").classList.add("active");
        setTimeout(() => {
          // 리다이렉트: 관리자 → /admin, 사용자 → /order
          window.location.href = result === "admin" ? "/admin" : "/order";
        }, 1500);
      }, 1500);
    } else {
      alert("로그인 실패!");
    }
  });
});