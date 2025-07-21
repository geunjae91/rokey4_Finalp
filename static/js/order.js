document.addEventListener("DOMContentLoaded", () => {
  const cartBtn = document.querySelector(".btn-outline-dark.flex-shrink-0");
  const inputQty = document.getElementById("inputQuantity");
  const cartCount = document.querySelector(".badge");

  cartBtn.addEventListener("click", () => {
    const qty = parseInt(inputQty.value);
    const maxQty = 1;

    if (qty > maxQty) {
      alert("수량이 없습니다.");
      return;
    }

    // 수량 1개는 허용
    fetch("/add_to_cart", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ product_name: "RC car", quantity: qty }),
    })
      .then((res) => res.json())
      .then((data) => {
        if (data.success) {
          // 카운트 업데이트
          let count = parseInt(cartCount.textContent);
          cartCount.textContent = count + qty;
        } else {
          alert("장바구니에 추가 실패");
        }
      });
  });
});
